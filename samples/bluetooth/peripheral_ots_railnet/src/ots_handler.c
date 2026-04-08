/*
 * ots_handler.c
 *
 * Gestion du pool d'objets OTS et callbacks Zephyr pour peripheral_ots_railnet.
 *
 * Rôle :
 *   - Maintenir un pool statique d'objets en RAM (pas d'allocation dynamique)
 *   - Implémenter les 6 callbacks OTS requis par la stack Zephyr
 *   - Déclencher le relay UART (SOTP) à la complétion d'une écriture BLE
 *   - Permettre l'ajout d'objets depuis l'iMX6 (sens iMX6 → BLE)
 *
 * Design du pool :
 *   CONFIG_BT_OTS_MAX_OBJ_CNT slots (3 par défaut dans prj.conf)
 *   Chaque slot : données brutes + nom + longueur écrite
 *   Mapping OTS ID → index de slot :
 *     idx = (id - BT_OTS_OBJ_ID_MIN) % CONFIG_BT_OTS_MAX_OBJ_CNT
 *   (même logique que le sample upstream peripheral_ots)
 *
 * Contrainte RAM nRF52840 :
 *   256 KB totaux, ~150 KB libres après stack BLE.
 *   3 objets × 32 KB = 96 KB → marge confortable.
 *
 * Auteur : STIMIO / équipe développement embarqué
 */

#include "ots_handler.h"
#include "uart_relay.h"

#include <string.h>
#include <stdio.h>
#include <zephyr/bluetooth/services/ots.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ots_handler, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------
 * Pool d'objets statique
 * ------------------------------------------------------------------------- */

/**
 * @brief Slot du pool d'objets.
 *
 * Chaque slot correspond à un objet OTS pouvant être stocké simultanément.
 * Le nom est un tableau fixe — pas de malloc sur nRF52840.
 */
struct ots_obj_slot {
	/** Données brutes de l'objet (taille fixe = OTS_HANDLER_OBJ_MAX_SIZE) */
	uint8_t  data[OTS_HANDLER_OBJ_MAX_SIZE];

	/** Nom de l'objet, NULL-terminé */
	char     name[CONFIG_BT_OTS_OBJ_MAX_NAME_LEN + 1];

	/** Octets effectivement écrits (peut être < taille allouée) */
	uint32_t written_len;

	/** true si ce slot est actuellement utilisé par un objet OTS */
	bool     in_use;
};

/*
 * Pool statique. Taille totale :
 *   CONFIG_BT_OTS_MAX_OBJ_CNT * sizeof(ots_obj_slot)
 *   ≈ 3 * (32768 + 24 + 4 + 1) ≈ 98 KB
 *
 * Ce tableau est en section .bss (initialisé à zéro par le runtime).
 */
static struct ots_obj_slot obj_pool[CONFIG_BT_OTS_MAX_OBJ_CNT];

/** Nombre de slots actuellement occupés */
static uint32_t obj_count;

/**
 * @brief Nom de l'objet en cours de création côté serveur.
 *
 * Positionné par ots_handler_add_object_from_uart() avant bt_ots_obj_add(),
 * lu par obj_created(), remis à NULL après.
 * NULL quand c'est le client BLE qui crée l'objet via OACP Create.
 */
static char const *pending_name;

/**
 * @brief Taille des données pré-remplies lors d'une création côté serveur.
 *
 * Positionné par ots_handler_add_object_from_uart() en même temps que pending_name.
 * Utilisé par obj_created() pour retourner size.cur = pending_data_len au lieu de 0.
 * Sans cela, Zephyr OTS voit l'objet comme vide (size.cur=0) et OLCP peut
 * ne pas le rendre accessible à la navigation.
 */
static uint32_t pending_data_len;

/** Instance OTS Zephyr — obtenue via bt_ots_free_instance_get() dans ots_handler_init() */
static struct bt_ots *ots_instance;

/* Macro de mapping ID → index de slot (identique au sample upstream) */
#define OBJ_ID_TO_IDX(id)  (((id) - BT_OTS_OBJ_ID_MIN) % (uint64_t)CONFIG_BT_OTS_MAX_OBJ_CNT)

/*
 * Work item pour différer bt_ots_obj_delete après retour de obj_write.
 *
 * bt_ots_obj_delete() exige que l'objet soit en BT_GATT_OTS_OBJECT_IDLE_STATE.
 * Or, la stack Zephyr OTS remet l'objet en IDLE *après* le retour de obj_write
 * (dans oacp_write_common, ligne 599). Un appel direct depuis obj_write retourne
 * -EBUSY. On différe donc la suppression via k_work pour qu'elle s'exécute après
 * que la stack a terminé son traitement.
 */
static uint64_t pending_delete_id;
static void delete_work_handler(struct k_work *work);
static K_WORK_DEFINE(delete_work, delete_work_handler);

static void delete_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	int err = bt_ots_obj_delete(ots_instance, pending_delete_id);

	if (err) {
		LOG_WRN("delete_work: bt_ots_obj_delete id=0x%012llx failed: %d",
			(unsigned long long)pending_delete_id, err);
	} else {
		LOG_DBG("delete_work: id=0x%012llx deleted", (unsigned long long)pending_delete_id);
	}
}

/* -------------------------------------------------------------------------
 * Callbacks OTS
 * ------------------------------------------------------------------------- */

/**
 * @brief Callback obj_created : appelé par la stack Zephyr OTS lors de la
 * création d'un objet (OACP Create depuis un client BLE, OU bt_ots_obj_add()
 * appelé en interne par ots_handler_add_object_from_uart).
 *
 * Responsabilité :
 *   - Vérifier que le pool a de la place
 *   - Allouer un slot (marquer in_use)
 *   - Remplir created_desc avec nom, taille, propriétés
 *
 * @param ots          Instance OTS.
 * @param conn         Connexion BLE (NULL si création côté serveur).
 * @param id           ID OTS auto-attribué par la stack.
 * @param add_param    Paramètres demandés (taille).
 * @param created_desc Descripteur à remplir par cette fonction.
 * @return 0 si succès, -ENOMEM si pool plein ou objet trop grand.
 */
static int obj_created(struct bt_ots *ots, struct bt_conn *conn, uint64_t id,
		       const struct bt_ots_obj_add_param *add_param,
		       struct bt_ots_obj_created_desc *created_desc)
{
	char id_str[BT_OTS_OBJ_ID_STR_LEN];
	uint64_t idx;

	bt_ots_obj_id_to_str(id, id_str, sizeof(id_str));

	if (obj_count >= ARRAY_SIZE(obj_pool)) {
		LOG_WRN("obj_created: pool full (id=%s, count=%u/%zu)",
			id_str, obj_count, ARRAY_SIZE(obj_pool));
		return -ENOMEM;
	}

	if (add_param->size > OTS_HANDLER_OBJ_MAX_SIZE) {
		LOG_WRN("obj_created: requested size %u > max %d (id=%s)",
			add_param->size, OTS_HANDLER_OBJ_MAX_SIZE, id_str);
		return -ENOMEM;
	}

	idx = OBJ_ID_TO_IDX(id);

	/* Initialiser le slot */
	memset(obj_pool[idx].data, 0, sizeof(obj_pool[idx].data));
	obj_pool[idx].written_len = 0;
	obj_pool[idx].in_use      = true;

	/*
	 * Nom de l'objet :
	 *   - Si création depuis ots_handler_add_object_from_uart() (pending_name non NULL) :
	 *     utiliser le nom fourni par l'iMX6.
	 *   - Si création depuis un client BLE (OACP Create) :
	 *     initialiser avec une chaîne vide (le client peut renommer via
	 *     Object Name Write si CONFIG_BT_OTS_OBJ_NAME_WRITE_SUPPORT=y).
	 */
	if (pending_name && strlen(pending_name) > 0) {
		strncpy(obj_pool[idx].name, pending_name,
			CONFIG_BT_OTS_OBJ_MAX_NAME_LEN);
		obj_pool[idx].name[CONFIG_BT_OTS_OBJ_MAX_NAME_LEN] = '\0';
	} else {
		obj_pool[idx].name[0] = '\0';
	}

	/* Remplir le descripteur retourné à la stack Zephyr OTS */
	created_desc->name       = obj_pool[idx].name;
	created_desc->size.alloc = add_param->size;

	/*
	 * size.cur :
	 *   - Création client BLE (OACP Create) : 0, les données arrivent ensuite via OACP Write.
	 *   - Création serveur (add_object_from_uart) : pending_data_len, car les données
	 *     sont copiées immédiatement après bt_ots_obj_add(). Sans cela, Zephyr OTS
	 *     voit l'objet comme vide et un client BLE ne peut pas le lire via OACP Read.
	 */
	created_desc->size.cur   = pending_name ? pending_data_len : 0;

	/* Propriétés : l'objet peut être lu, écrit (avec patch), supprimé */
	created_desc->props = 0;
	BT_OTS_OBJ_SET_PROP_READ(created_desc->props);
	BT_OTS_OBJ_SET_PROP_WRITE(created_desc->props);
	BT_OTS_OBJ_SET_PROP_PATCH(created_desc->props);
	BT_OTS_OBJ_SET_PROP_DELETE(created_desc->props);

	obj_count++;

	LOG_INF("obj_created: id=%s idx=%llu size_alloc=%u name='%s' (pool: %u/%zu)",
		id_str, (unsigned long long)idx, add_param->size,
		obj_pool[idx].name, obj_count, ARRAY_SIZE(obj_pool));

	return 0;
}

/**
 * @brief Callback obj_deleted : appelé quand OACP Delete est reçu,
 * ou quand bt_ots_obj_delete() est appelé en interne.
 *
 * Libère le slot du pool.
 */
static int obj_deleted(struct bt_ots *ots, struct bt_conn *conn, uint64_t id)
{
	char id_str[BT_OTS_OBJ_ID_STR_LEN];
	uint64_t idx;

	bt_ots_obj_id_to_str(id, id_str, sizeof(id_str));
	idx = OBJ_ID_TO_IDX(id);

	if (!obj_pool[idx].in_use) {
		LOG_WRN("obj_deleted: slot %llu already free (id=%s)",
			(unsigned long long)idx, id_str);
		return 0;
	}

	obj_pool[idx].in_use      = false;
	obj_pool[idx].written_len = 0;
	obj_count--;

	LOG_INF("obj_deleted: id=%s idx=%llu (pool: %u/%zu)",
		id_str, (unsigned long long)idx,
		obj_count, ARRAY_SIZE(obj_pool));

	return 0;
}

/**
 * @brief Callback obj_selected : appelé quand OLCP GoTo/First/Last/Next/Prev
 * sélectionne un objet. Informatif seulement.
 */
static void obj_selected(struct bt_ots *ots, struct bt_conn *conn, uint64_t id)
{
	char id_str[BT_OTS_OBJ_ID_STR_LEN];

	bt_ots_obj_id_to_str(id, id_str, sizeof(id_str));
	LOG_DBG("obj_selected: id=%s", id_str);
}

/**
 * @brief Callback obj_read : appelé en boucle par Zephyr OTS lors d'une
 * lecture OACP Read. Zephyr demande des fragments successifs.
 *
 * Fonctionnement :
 *   - data == NULL → fin de l'opération de lecture (callback informatif)
 *   - data != NULL → fournir le prochain fragment
 *
 * On pointe directement dans le buffer statique (zero-copy).
 *
 * @param data   Out: pointeur vers le fragment suivant. NULL = fin de lecture.
 * @param len    Longueur maximale demandée pour ce fragment.
 * @param offset Offset dans l'objet.
 * @return Taille du fragment à envoyer (≤ len), ou négatif en cas d'erreur.
 */
static ssize_t obj_read(struct bt_ots *ots, struct bt_conn *conn, uint64_t id,
			void **data, size_t len, off_t offset)
{
	uint64_t idx = OBJ_ID_TO_IDX(id);

	if (!data) {
		/* Fin de l'opération de lecture — callback informatif */
		LOG_DBG("obj_read: completed for id=0x%012llx",
			(unsigned long long)id);
		return 0;
	}

	if (!obj_pool[idx].in_use) {
		LOG_ERR("obj_read: slot %llu not in use (id=0x%012llx)",
			(unsigned long long)idx, (unsigned long long)id);
		return -ENOENT;
	}

	if ((uint32_t)offset >= obj_pool[idx].written_len) {
		return 0; /* Fin des données */
	}

	/* Pointer directement dans le buffer — zero-copy vers la stack BLE */
	*data = &obj_pool[idx].data[offset];

	/* Retourner min(len demandé, données disponibles depuis offset) */
	size_t available = obj_pool[idx].written_len - (uint32_t)offset;

	return (ssize_t)MIN(len, available);
}

/**
 * @brief Callback obj_write : appelé en boucle par Zephyr OTS lors d'une
 * écriture OACP Write. Les données arrivent en fragments.
 *
 * Comportement clé :
 *   Quand remaining == 0, c'est le dernier fragment → l'objet est complet.
 *   On déclenche alors uart_relay_send_object() pour relayer vers l'iMX6.
 *
 * @param data      Fragment de données entrant.
 * @param len       Taille du fragment.
 * @param offset    Offset dans l'objet.
 * @param remaining Octets restants après ce fragment (0 = dernier fragment).
 * @return Taille du fragment acceptée (= len), ou négatif en cas d'erreur.
 */
static ssize_t obj_write(struct bt_ots *ots, struct bt_conn *conn, uint64_t id,
			 const void *data, size_t len, off_t offset,
			 size_t remaining)
{
	uint64_t idx = OBJ_ID_TO_IDX(id);

	if (!obj_pool[idx].in_use) {
		LOG_ERR("obj_write: slot %llu not in use (id=0x%012llx)",
			(unsigned long long)idx, (unsigned long long)id);
		return -ENOENT;
	}

	/* Vérifier que le fragment ne déborde pas le buffer alloué */
	if ((size_t)offset + len > OTS_HANDLER_OBJ_MAX_SIZE) {
		LOG_ERR("obj_write: overflow (offset=%ld + len=%zu > max=%d)",
			(long)offset, len, OTS_HANDLER_OBJ_MAX_SIZE);
		return -ENOMEM;
	}

	/* Copier le fragment dans le buffer */
	memcpy(&obj_pool[idx].data[offset], data, len);
	obj_pool[idx].written_len = (uint32_t)(offset + len);

	LOG_DBG("obj_write: id=0x%012llx offset=%ld len=%zu remaining=%zu",
		(unsigned long long)id, (long)offset, len, remaining);

	/* Dernier fragment : objet complet → relayer vers iMX6 via SOTP, puis libérer */
	if (remaining == 0) {
		LOG_INF("obj_write: object complete id=0x%012llx total=%u bytes → UART relay",
			(unsigned long long)id, obj_pool[idx].written_len);

		int err = uart_relay_send_object(obj_pool[idx].data,
						 obj_pool[idx].written_len);
		if (err) {
			/*
			 * Ne pas retourner d'erreur au client BLE :
			 * l'écriture OTS a réussi côté nRF52840.
			 * L'échec du relay UART est loggé pour debug.
			 */
			LOG_ERR("obj_write: uart_relay_send_object failed: %d", err);
		}

		/* Différer bt_ots_obj_delete via k_work.
		 * Appel direct ici → -EBUSY : la stack Zephyr remet l'objet en
		 * IDLE_STATE après le retour de ce callback (oacp_write_common:599).
		 * Le work handler s'exécute après que la stack a terminé.
		 */
		pending_delete_id = id;
		k_work_submit(&delete_work);
	}

	return (ssize_t)len;
}

/**
 * @brief Callback obj_cal_checksum : appelé lors d'un OACP Calculate Checksum.
 *
 * Fournit un pointeur vers les données de l'objet pour que la stack Zephyr
 * calcule le CRC32. La stack gère le calcul — on ne fait que pointer les données.
 *
 * @param data   Out: pointeur vers les données à partir de offset.
 * @return 0 si succès, -ENOENT si slot non utilisé.
 */
static int obj_cal_checksum(struct bt_ots *ots, struct bt_conn *conn,
			    uint64_t id, off_t offset, size_t len, void **data)
{
	uint64_t idx = OBJ_ID_TO_IDX(id);

	if (!obj_pool[idx].in_use) {
		return -ENOENT;
	}

	*data = &obj_pool[idx].data[offset];
	return 0;
}

/**
 * @brief Callback obj_name_written : appelé quand un client BLE renomme un objet.
 *
 * Requis quand CONFIG_BT_OTS_OBJ_NAME_WRITE_SUPPORT=y (dépendance de CREATE).
 * On log le changement — le pool est déjà mis à jour par la stack Zephyr.
 */
static void obj_name_written(struct bt_ots *ots, struct bt_conn *conn,
			     uint64_t id, const char *cur_name,
			     const char *new_name)
{
	char id_str[BT_OTS_OBJ_ID_STR_LEN];

	bt_ots_obj_id_to_str(id, id_str, sizeof(id_str));
	LOG_INF("obj_name_written: id=%s '%s' → '%s'", id_str, cur_name, new_name);
}

/* -------------------------------------------------------------------------
 * Structure de callbacks OTS
 * Passée à bt_ots_init().
 * ------------------------------------------------------------------------- */
static struct bt_ots_cb ots_callbacks = {
	.obj_created      = obj_created,
	.obj_deleted      = obj_deleted,
	.obj_selected     = obj_selected,
	.obj_read         = obj_read,
	.obj_write        = obj_write,
	.obj_cal_checksum = obj_cal_checksum,
	.obj_name_written = obj_name_written,
};

/* -------------------------------------------------------------------------
 * API publique
 * ------------------------------------------------------------------------- */

int ots_handler_init(void)
{
	struct bt_ots_init_param init;

	/*
	 * bt_ots_free_instance_get() retourne un pointeur vers une instance
	 * pré-allouée dans le pool Zephyr (taille = CONFIG_BT_OTS_MAX_INST_CNT,
	 * défini automatiquement à 1 quand CONFIG_BT_OTS=y).
	 */
	ots_instance = bt_ots_free_instance_get();
	if (!ots_instance) {
		LOG_ERR("bt_ots_free_instance_get failed — pool épuisé");
		return -ENOMEM;
	}

	/* Initialiser le descripteur à zéro puis configurer les features */
	memset(&init, 0, sizeof(init));

	/* OACP features : opérations supportées sur les objets */
	BT_OTS_OACP_SET_FEAT_CREATE(init.features.oacp);
	BT_OTS_OACP_SET_FEAT_DELETE(init.features.oacp);
	BT_OTS_OACP_SET_FEAT_READ(init.features.oacp);
	BT_OTS_OACP_SET_FEAT_WRITE(init.features.oacp);
	BT_OTS_OACP_SET_FEAT_PATCH(init.features.oacp);
	BT_OTS_OACP_SET_FEAT_CHECKSUM(init.features.oacp);

	/* OLCP features : navigation dans la liste d'objets */
	BT_OTS_OLCP_SET_FEAT_GO_TO(init.features.olcp);

	init.cb = &ots_callbacks;

	int err = bt_ots_init(ots_instance, &init);

	if (err) {
		LOG_ERR("bt_ots_init failed: %d", err);
		return err;
	}

	LOG_INF("OTS server ready (pool: %zu objects × %d KB each)",
		ARRAY_SIZE(obj_pool),
		OTS_HANDLER_OBJ_MAX_SIZE / 1024);

	return 0;
}

struct bt_ots *ots_get_instance(void)
{
	return ots_instance;
}

void ots_handler_cleanup_on_disconnect(void)
{
	/*
	 * Supprimer chaque objet de la stack Zephyr OTS via bt_ots_obj_delete().
	 * Sans cela, la stack OTS garde ses objets internes même si on remet
	 * obj_pool à zéro, et bt_le_adv_start échoue avec -ENOMEM au reconnect.
	 *
	 * L'ID d'un objet d'index i est : BT_OTS_OBJ_ID_MIN + i
	 * (mapping inverse de OBJ_ID_TO_IDX).
	 */
	for (size_t i = 0; i < ARRAY_SIZE(obj_pool); i++) {
		if (obj_pool[i].in_use && ots_instance) {
			uint64_t id = BT_OTS_OBJ_ID_MIN + (uint64_t)i;
			int err = bt_ots_obj_delete(ots_instance, id);
			if (err && err != -ENOENT) {
				LOG_WRN("cleanup: bt_ots_obj_delete idx=%zu err=%d", i, err);
			}
		}
		obj_pool[i].in_use      = false;
		obj_pool[i].written_len = 0;
		obj_pool[i].name[0]     = '\0';
	}
	obj_count = 0;
	LOG_INF("ots_handler_cleanup_on_disconnect: pool réinitialisé");
}

int ots_handler_add_object_from_uart(const uint8_t *data, size_t len,
				     const char *name)
{
	struct bt_ots_obj_add_param param;
	int err;

	if (!ots_instance) {
		LOG_ERR("ots_handler_add_object_from_uart: OTS not initialized");
		return -EINVAL;
	}

	if (obj_count >= ARRAY_SIZE(obj_pool)) {
		LOG_WRN("ots_handler_add_object_from_uart: pool full");
		return -ENOMEM;
	}

	if (len > OTS_HANDLER_OBJ_MAX_SIZE) {
		LOG_ERR("ots_handler_add_object_from_uart: len %zu > max %d",
			len, OTS_HANDLER_OBJ_MAX_SIZE);
		return -ENOMEM;
	}

	/*
	 * Passer le nom et les données via les variables globales temporaires.
	 * Le callback obj_created() sera appelé de manière synchrone dans
	 * bt_ots_obj_add() et lira pending_name.
	 *
	 * Note : cette approche n'est pas thread-safe si plusieurs threads
	 * appellent ots_handler_add_object_from_uart() en parallèle.
	 * Dans notre cas, seul le thread sotp_rx appelle cette fonction → OK.
	 */
	pending_name     = name;
	pending_data_len = (uint32_t)len;

	param.size           = (uint32_t)len;
	param.type.uuid.type = BT_UUID_TYPE_16;
	param.type.uuid_16.val = BT_UUID_OTS_TYPE_UNSPECIFIED_VAL;

	err = bt_ots_obj_add(ots_instance, &param);

	pending_name     = NULL;
	pending_data_len = 0;

	if (err < 0) {
		LOG_ERR("bt_ots_obj_add failed: %d", err);
		return err;
	}

	/*
	 * bt_ots_obj_add a déclenché obj_created() qui a alloué le slot.
	 * Maintenant on copie les données dans le slot via le pool.
	 * On retrouve l'index via obj_count - 1 n'est pas fiable (race),
	 * mais l'ID retourné par bt_ots_obj_add (err ≥ 0 = ID en cas de succès)
	 * n'est pas documenté ainsi dans cette version — on utilise le dernier
	 * slot alloué via pending_name qui a été mis à jour dans obj_created().
	 *
	 * En pratique : le dernier slot alloué est (obj_count-1) dans l'ordre
	 * d'attribution. On parcourt le pool pour trouver le slot avec le bon nom.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(obj_pool); i++) {
		if (obj_pool[i].in_use &&
		    obj_pool[i].written_len == 0 &&
		    name && strcmp(obj_pool[i].name, name) == 0) {
			memcpy(obj_pool[i].data, data, len);
			obj_pool[i].written_len = (uint32_t)len;
			LOG_INF("ots_handler_add_object_from_uart: "
				"added slot=%zu len=%zu name='%s'",
				i, len, name);
			return 0;
		}
	}

	LOG_ERR("ots_handler_add_object_from_uart: could not find allocated slot");
	return -ENOENT;
}

/*
 * peripheral_ots_railnet/src/main.c
 *
 * Firmware Zephyr OTS pour NINA-B301 (nRF52840) sur RailNet200.
 *
 * Ce firmware expose un serveur GATT OTS (Object Transfer Service, UUID 0x1825)
 * permettant à un client Android de transférer des objets (fichiers, données)
 * vers l'iMX6 via BLE, sans nécessiter de Bluetooth Classic (BR/EDR).
 *
 * Pourquoi OTS et pas OBEX ?
 *   Le nRF52840 est BLE uniquement (pas de circuit radio BR/EDR).
 *   OBEX nécessite BR/EDR → L2CAP → RFCOMM → GOEP → OBEX.
 *   OTS est le remplaçant BLE standardisé (UUID 0x1825, Bluetooth SIG 2017).
 *   Voir docs/ble-obex-bredr-investigation.md pour l'analyse complète.
 *
 * Architecture des couches :
 *   Android OTS Central
 *       ↕ BLE (GATT + L2CAP CoC)
 *   NINA-B301 : ce firmware
 *       ├── ots_handler.c : pool d'objets + callbacks OTS
 *       └── uart_relay.c  : protocole SOTP vers /dev/ttymxc7 (iMX6)
 *
 * Séquence d'initialisation :
 *   1. bt_enable(bt_ready_cb)         — démarre la stack BLE Zephyr host
 *   2. bt_ready_cb()
 *       a. ots_handler_init()         — enregistre le serveur GATT OTS
 *       b. uart_relay_init()          — initialise l'UART SOTP vers iMX6
 *       c. bt_le_adv_start()          — démarre l'advertising BLE
 *   3. Boucle principale idle
 *   4. Thread stats toutes les 10s (visible via JLinkRTTClient)
 *
 * Auteur : STIMIO / équipe développement embarqué
 * Board  : railnet200_nina_b301/nrf52840
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/ots.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "ots_handler.h"
#include "uart_relay.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Work item pour différer bt_le_adv_start après disconnect.
 * Le callback disconnected() est appelé depuis le thread BT alors que
 * la stack OTS n'a pas encore libéré ses ressources internes.
 * En différant via k_work, on laisse la stack finir son cleanup avant
 * de relancer l'advertising.
 */
static struct k_work adv_restart_work;

/* -------------------------------------------------------------------------
 * Advertising data
 *
 * ad[]  = données primary advertising (envoyées dans chaque paquet ADV_IND)
 *   - Flags : General Discoverable | No BR/EDR (nRF52840 = LE only)
 *
 * sd[]  = données scan response (envoyées en réponse à un SCAN_REQ)
 *   - UUID OTS 0x1825 en 16 bits little-endian
 *   - Nom complet du device (CONFIG_BT_DEVICE_NAME = "RailNet200")
 *
 * Note : séparer UUID et Name dans sd[] permet de garder ad[] minimal
 * et de rester dans les 31 octets max par paquet.
 * ------------------------------------------------------------------------- */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS,
		      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
};

static const struct bt_data sd[] = {
	/* UUID OTS = 0x1825, encodé little-endian en 2 octets */
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_OTS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* -------------------------------------------------------------------------
 * Statistiques atomiques
 * Lues par le thread stats (preempt 10), écrites par les callbacks BLE (BT RX).
 * Utilisation d'atomiques pour éviter les races sans mutex.
 * ------------------------------------------------------------------------- */
static atomic_t stat_ble_connections;
static atomic_t stat_objects_received;

/* -------------------------------------------------------------------------
 * Callbacks de connexion BLE
 *
 * Macro BT_CONN_CB_DEFINE enregistre les callbacks sans bt_conn_cb_register().
 * C'est l'API moderne Zephyr (évite les problèmes d'ordre d'initialisation).
 * ------------------------------------------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed: err=%u (%s)",
			err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s", addr);
	atomic_inc(&stat_ble_connections);

	/* Initier DLE (Data Length Extension) : demande au peer d'utiliser des
	 * PDUs LL de 251 bytes au lieu de 27 bytes (défaut sans DLE).
	 * Sans DLE, un PDU L2CAP CoC de 251 bytes serait fragmenté en 10 LL PDUs
	 * de 27 bytes, dépassant le buffer ACL RX de 255 bytes au 10ème fragment.
	 * Le contrôleur nRF52840 Zephyr ne négocie pas DLE automatiquement,
	 * donc on l'initie explicitement ici.
	 */
	int dle_err = bt_conn_le_data_len_update(conn,
		BT_CONN_LE_DATA_LEN_PARAM(BT_GAP_DATA_LEN_MAX,
					  BT_GAP_DATA_TIME_MAX));
	if (dle_err) {
		LOG_WRN("Failed to initiate DLE: %d", dle_err);
	} else {
		LOG_INF("DLE update requested (tx_max_len=%u)", BT_GAP_DATA_LEN_MAX);
	}
}

static void adv_restart_work_handler(struct k_work *work)
{
	int err;

	ots_handler_cleanup_on_disconnect();

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			      ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("bt_le_adv_start after disconnect failed: %d", err);
	} else {
		LOG_INF("Advertising restarted");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s, reason=%u (%s)",
		addr, reason, bt_hci_err_to_str(reason));
	atomic_dec(&stat_ble_connections);

	/* Différer le restart advertising pour laisser la stack OTS
	 * finir son propre cleanup (ots_conn_disconnected) avant d'appeler
	 * bt_le_adv_start, sinon -ENOMEM car ressources pas encore libérées.
	 */
	k_work_submit(&adv_restart_work);
}

static void le_data_len_updated(struct bt_conn *conn,
				struct bt_conn_le_data_len_info *info)
{
	LOG_INF("DLE negotiated: tx_max_len=%u tx_max_time=%u "
		"rx_max_len=%u rx_max_time=%u",
		info->tx_max_len, info->tx_max_time,
		info->rx_max_len, info->rx_max_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected         = connected,
	.disconnected      = disconnected,
	.le_data_len_updated = le_data_len_updated,
};

/* -------------------------------------------------------------------------
 * bt_ready_cb
 *
 * Appelé par la stack Zephyr quand bt_enable() a terminé l'initialisation.
 *
 * Ordre d'appel critique :
 *   1. ots_handler_init() AVANT bt_le_adv_start()
 *      → Le service GATT OTS doit être enregistré avant qu'un central
 *        puisse se connecter et découvrir les services.
 *   2. uart_relay_init() peut être fait en parallèle (indépendant du BLE).
 * ------------------------------------------------------------------------- */
static void bt_ready_cb(int err)
{
	int ret;

	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	k_work_init(&adv_restart_work, adv_restart_work_handler);

	/* Initialiser le pool d'objets et le serveur GATT OTS */
	ret = ots_handler_init();
	if (ret) {
		LOG_ERR("ots_handler_init failed: %d", ret);
		return;
	}

	/* Initialiser le relay UART SOTP vers iMX6 */
	ret = uart_relay_init();
	if (ret) {
		LOG_ERR("uart_relay_init failed: %d", ret);
		return;
	}

	/*
	 * Démarrer l'advertising BLE.
	 *
	 * BT_LE_ADV_CONN_FAST_1 : connectable undirected, interval 30-60 ms.
	 * (Valeur recommandée GAP "fast interval 1" pour découverte rapide.)
	 *
	 * Note : avec hci_uart, on forçait 20ms via des commandes HCI raw.
	 * Ici on utilise directement le paramètre Zephyr host — plus propre.
	 */
	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			      ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (ret) {
		LOG_ERR("bt_le_adv_start failed: %d", ret);
		return;
	}

	LOG_INF("Advertising started — name: \"%s\", UUID OTS: 0x%04X",
		CONFIG_BT_DEVICE_NAME, BT_UUID_OTS_VAL);
}

/* -------------------------------------------------------------------------
 * Thread de statistiques
 *
 * Affiche toutes les 10s via RTT (visible dans JLinkRTTClient).
 * Permet de vérifier le bon fonctionnement sans moniteur série.
 *
 * Priorité preempt(10) — moins prioritaire que le traitement BLE/UART.
 * Stack 512 octets — suffisant pour LOG_INF + atomic_get.
 * ------------------------------------------------------------------------- */
static void stats_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sleep(K_SECONDS(10));
		LOG_INF("Stats: BLE connections=%ld | objects received=%ld",
			(long)atomic_get(&stat_ble_connections),
			(long)atomic_get(&stat_objects_received));
	}
}

K_THREAD_DEFINE(stats_thread, 512,
		stats_thread_fn, NULL, NULL, NULL,
		K_PRIO_PREEMPT(10), 0, 0);

/* -------------------------------------------------------------------------
 * main()
 *
 * BD Address fixe : DE:CA:F1:5B:AD:11 (Random Static Address)
 *
 * bt_id_create() doit être appelé AVANT bt_enable(). Il crée une identité
 * BLE avec une adresse Random Static fixe. Les bits[47:46] doivent être 0b11
 * (obligatoire par la spec BT pour une Random Static Address).
 * DE:CA:F1:5B:AD:11 → octet de poids fort = 0xDE = 1101 1110 → bits[7:6]=11 ✓
 *
 * L'adresse est stockée en little-endian dans bt_addr_t.val :
 * index 0 = octet de poids faible (0x11), index 5 = octet de poids fort (0xDE).
 * ------------------------------------------------------------------------- */
int main(void)
{
	int err;
	bt_addr_le_t addr = {
		.type = BT_ADDR_LE_RANDOM,
		.a.val = { 0x11, 0xAD, 0x5B, 0xF1, 0xCA, 0xDE },
	};

	LOG_INF("peripheral_ots_railnet starting...");

	/* Fixer l'adresse BLE avant d'activer la stack */
	err = bt_id_create(&addr, NULL);
	if (err < 0) {
		LOG_ERR("bt_id_create failed: %d", err);
		return err;
	}
	LOG_INF("BD Address: DE:CA:F1:5B:AD:11 (Random Static)");

	/*
	 * bt_enable() est asynchrone : la stack s'initialise en tâche de fond,
	 * puis appelle bt_ready_cb() quand elle est prête.
	 * On ne bloque pas ici — la boucle while(1) ci-dessous est idle.
	 */
	err = bt_enable(bt_ready_cb);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return err;
	}

	/*
	 * Boucle principale idle.
	 * Tout le traitement se fait dans :
	 *   - bt_ready_cb() et ses appels (thread système BT)
	 *   - Callbacks OTS (thread BT RX)
	 *   - Thread uart_rx (sotp_rx, défini dans uart_relay.c)
	 *   - Thread stats (défini ci-dessus)
	 */
	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}

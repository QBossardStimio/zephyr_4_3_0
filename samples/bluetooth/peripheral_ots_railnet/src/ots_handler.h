/*
 * ots_handler.h
 *
 * Pool d'objets statique et callbacks OTS pour le firmware peripheral_ots_railnet.
 *
 * Interface publique utilisée par main.c :
 *   - ots_handler_init() : initialise le serveur OTS et le pool d'objets
 *   - ots_get_instance()  : retourne le pointeur bt_ots pour bt_ots_obj_add() etc.
 *   - ots_handler_add_object_from_uart() : ajoute un objet reçu de l'iMX6
 */

#ifndef OTS_HANDLER_H_
#define OTS_HANDLER_H_

#include <stdint.h>
#include <stddef.h>
#include <zephyr/bluetooth/services/ots.h>

/**
 * @brief Taille maximale d'un objet OTS en RAM (32 KB).
 *
 * Ajuster selon la RAM disponible après mesure avec west build --ram-report.
 * Contrainte nRF52840 : 256 KB totaux, ~100-150 KB disponibles après stack BLE.
 * Avec CONFIG_BT_OTS_MAX_OBJ_CNT=3 : 3 * 32KB = 96 KB.
 */
#define OTS_HANDLER_OBJ_MAX_SIZE  (32 * 1024)

/**
 * @brief Initialise le pool d'objets et enregistre le serveur OTS.
 *
 * Enregistre les callbacks OTS auprès de la stack Zephyr.
 * Doit être appelé depuis bt_ready_cb(), après bt_enable().
 *
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int ots_handler_init(void);

/**
 * @brief Retourne l'instance OTS Zephyr.
 *
 * Utilisé par main.c si besoin d'appeler bt_ots_obj_add() manuellement.
 *
 * @return Pointeur vers struct bt_ots, ou NULL si non initialisé.
 */
struct bt_ots *ots_get_instance(void);

/**
 * @brief Callback appelé par uart_relay quand l'iMX6 envoie un objet vers BLE.
 *
 * Ajoute l'objet dans le pool OTS pour qu'un client BLE puisse le lire.
 *
 * @param data  Données de l'objet.
 * @param len   Longueur des données.
 * @param name  Nom de l'objet (chaîne NULL-terminée, max BT_OTS_OBJ_MAX_NAME_LEN).
 * @return 0 si succès, -ENOMEM si pool plein.
 */
int ots_handler_add_object_from_uart(const uint8_t *data, size_t len,
				     const char *name);

/**
 * @brief Remet le pool d'objets à zéro après une déconnexion BLE.
 *
 * Appelé depuis main.c::disconnected() pour libérer les slots qui
 * n'ont pas été explicitement supprimés via OACP Delete.
 * Permet à bt_le_adv_start() de réussir (évite ENOMEM).
 */
void ots_handler_cleanup_on_disconnect(void);

#endif /* OTS_HANDLER_H_ */

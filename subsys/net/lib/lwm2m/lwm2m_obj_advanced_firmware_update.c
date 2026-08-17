/*
 * Copyright (c) 2026 Vilhelm Engström
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <time.h>

#include <zephyr/logging/log.h>

#include "lwm2m_object.h"
#include "lwm2m_engine.h"
#include "lwm2m_pull_context.h"

LOG_MODULE_REGISTER(net_lwm2m_obj_adv_fw_update, CONFIG_LWM2M_LOG_LEVEL);

#define ADV_FW_VERSION_MAJOR 1
#define ADV_FW_VERSION_MINOR 0

BUILD_ASSERT(CONFIG_LWM2M_ADVANCED_FIRMWARE_UPDATE_OBJ_INSTANCE_COUNT > 0);
#define MAX_INSTANCE_COUNT CONFIG_LWM2M_ADVANCED_FIRMWARE_UPDATE_OBJ_INSTANCE_COUNT

/* Advanced firmware update package resource IDs */
#define ADV_FW_PKG_ID                    0
#define ADV_FW_PKG_URI_ID                1
#define ADV_FW_UPDATE_ID                 2
#define ADV_FW_STATE_ID                  3
#define ADV_FW_UPDATE_RESULT_ID          5
#define ADV_FW_PKG_NAME_ID               6
#define ADV_FW_PKG_VERSION_ID            7
#define ADV_FW_UPDATE_PROTO_SUPPORT_ID   8
#define ADV_FW_DELIVERY_METHOD_ID        9
#define ADV_FW_CANCEL_ID                 10
#define ADV_FW_SEVERITY_ID               11
#define ADV_FW_LAST_STATE_CHANGE_TIME_ID 12
#define ADV_FW_MAX_DEFER_PERIOD_ID       13
#define ADV_FW_COMPONENT_NAME_ID         14
#define ADV_FW_CURRENT_VERSION_ID        15

#define ADV_FW_MAX_ID 16

/* /33629/x/2 and /33620/x/10 are execute */
#define RESOURCE_INSTANCE_COUNT (ADV_FW_MAX_ID - 2)

enum {
	ADV_FW_DELIVERY_PULL_ONLY,
	ADV_FW_DELIVERY_PUSH_ONLY,
	ADV_FW_DELIVERY_BOTH,
};

static uint8_t adv_fw_update_state[MAX_INSTANCE_COUNT];
static uint8_t adv_fw_update_result[ARRAY_SIZE(adv_fw_update_state)];
static char adv_fw_update_pkg_uri[MAX_INSTANCE_COUNT][LWM2M_PACKAGE_URI_LEN];
static uint8_t adv_fw_update_delivery_method[ARRAY_SIZE(adv_fw_update_state)];
static uint8_t adv_fw_update_severity[ARRAY_SIZE(adv_fw_update_state)];
static time_t adv_fw_update_last_state_chg_time[ARRAY_SIZE(adv_fw_update_state)];
static uint32_t adv_fw_update_max_defer_period[ARRAY_SIZE(adv_fw_update_state)];

static struct lwm2m_engine_obj adv_fw_update;
static struct lwm2m_engine_obj_field fields[] = {
	/* /33629/x/0 */
	OBJ_FIELD_DATA(ADV_FW_PKG_ID, W, OPAQUE),
	/* /33629/x/1 */
	OBJ_FIELD_DATA(ADV_FW_PKG_URI_ID, RW, STRING),
	/* /33629/x/2 */
	OBJ_FIELD_EXECUTE(ADV_FW_UPDATE_ID),
	/* /33629/x/3 */
	OBJ_FIELD_DATA(ADV_FW_STATE_ID, R, U8),
	/* /33629/x/5 */
	OBJ_FIELD_DATA(ADV_FW_UPDATE_RESULT_ID, R, U8),
	/* /33629/x/6 */
	OBJ_FIELD_DATA(ADV_FW_PKG_NAME_ID, R_OPT, STRING),
	/* /33629/x/7 */
	OBJ_FIELD_DATA(ADV_FW_PKG_VERSION_ID, R_OPT, STRING),
	/* /33629/x/8 */
	OBJ_FIELD_DATA(ADV_FW_UPDATE_PROTO_SUPPORT_ID, R_OPT, U8),
	/* /33629/x/9 */
	OBJ_FIELD_DATA(ADV_FW_DELIVERY_METHOD_ID, R, U8),
	/* /33629/x/10 */
	OBJ_FIELD_EXECUTE_OPT(ADV_FW_CANCEL_ID),
	/* /33629/x/11 */
	OBJ_FIELD_DATA(ADV_FW_SEVERITY_ID, RW_OPT, U8),
	/* /33629/x/12 */
	OBJ_FIELD_DATA(ADV_FW_LAST_STATE_CHANGE_TIME_ID, R_OPT, TIME),
	/* /33629/x/13 */
	OBJ_FIELD_DATA(ADV_FW_MAX_DEFER_PERIOD_ID, RW_OPT, U32),
	/* /33629/x/14 */
	OBJ_FIELD_DATA(ADV_FW_COMPONENT_NAME_ID, R_OPT, STRING),
	/* /33629/x/15 */
	OBJ_FIELD_DATA(ADV_FW_CURRENT_VERSION_ID, R, STRING),
	/* Multi-instance OBJLINK resources /33629/x/16 and /33629/x/17 omitted for now
	 */
};

static struct lwm2m_engine_obj_inst inst[ARRAY_SIZE(adv_fw_update_state)];
static struct lwm2m_engine_res res[ARRAY_SIZE(adv_fw_update_state)][ADV_FW_MAX_ID];
static struct lwm2m_engine_res_inst res_inst[ARRAY_SIZE(adv_fw_update_state)]
					    [RESOURCE_INSTANCE_COUNT];

static lwm2m_engine_set_data_cb_t adv_fw_write_cbs[ARRAY_SIZE(adv_fw_update_state)];
static lwm2m_engine_execute_cb_t adv_fw_update_cbs[ARRAY_SIZE(adv_fw_update_state)];
static lwm2m_engine_user_cb_t adv_fw_update_cancel_cbs[ARRAY_SIZE(adv_fw_update_state)];

uint8_t lwm2m_adv_fw_get_update_state_inst(uint16_t obj_inst_id)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_update_state))) {
		LOG_ERR("Invalid object instance %" PRIu16 ", have %zu", obj_inst_id,
			ARRAY_SIZE(adv_fw_update_state));
		return STATE_IDLE;
	}

	return adv_fw_update_state[obj_inst_id];
}

int lwm2m_adv_fw_set_write_cb_inst(uint16_t obj_inst_id, lwm2m_engine_set_data_cb_t cb)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_write_cbs))) {
		return -EINVAL;
	}

	adv_fw_write_cbs[obj_inst_id] = cb;
	return 0;
}

lwm2m_engine_set_data_cb_t lwm2m_adv_fw_get_write_cb_inst(uint16_t obj_inst_id)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_write_cbs))) {
		return NULL;
	}

	return adv_fw_write_cbs[obj_inst_id];
}

int lwm2m_adv_fw_set_update_cb_inst(uint16_t obj_inst_id, lwm2m_engine_execute_cb_t cb)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_update_cbs))) {
		return -EINVAL;
	}

	adv_fw_update_cbs[obj_inst_id] = cb;
	return 0;
}

lwm2m_engine_execute_cb_t lwm2m_adv_fw_get_update_cb_inst(uint16_t obj_inst_id)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_update_cbs))) {
		return NULL;
	}

	return adv_fw_update_cbs[obj_inst_id];
}

int lwm2m_adv_fw_set_cancel_cb_inst(uint16_t obj_inst_id, lwm2m_engine_user_cb_t cb)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_update_cancel_cbs))) {
		return -EINVAL;
	}

	adv_fw_update_cancel_cbs[obj_inst_id] = cb;
	return 0;
}

lwm2m_engine_user_cb_t lwm2m_adv_fw_get_cancel_cb_inst(uint16_t obj_inst_id)
{
	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_update_cancel_cbs))) {
		return NULL;
	}

	return adv_fw_update_cancel_cbs[obj_inst_id];
}

static void lwm2m_adv_fw_set_update_state_inst(uint16_t obj_inst_id, uint8_t state)
{
	int ret;
	uint8_t prev_state;
	struct lwm2m_obj_path path = LWM2M_OBJ(LWM2M_OBJECT_ADVANCED_FIRMWARE_UPDATE_ID,
					       obj_inst_id, ADV_FW_UPDATE_RESULT_ID);

	ret = -EINVAL;
	lwm2m_registry_lock();

	prev_state = adv_fw_update_state[obj_inst_id];

	switch ((prev_state << 8) | state) {
	case (STATE_IDLE << 8) | STATE_DOWNLOADING: /* Idle -> downloading */
		ret = lwm2m_set_u8(&path, RESULT_DEFAULT);
		break;
	case (STATE_DOWNLOADING << 8) | STATE_DOWNLOADED: /* Downloading -> downloaded */
		ret = lwm2m_set_u8(&path, RESULT_DEFAULT);
		break;
	case (STATE_UPDATING << 8) | STATE_DOWNLOADED: /* Updating -> downloaded */
		ret = lwm2m_set_u8(&path, RESULT_UPDATE_FAILED);
		break;
	case (STATE_DOWNLOADED << 8) | STATE_UPDATING: /* Downloaded -> updating */
		ret = 0;
		break;
	case (STATE_IDLE << 8) | STATE_IDLE:        /* Idle -> idle */
	case (STATE_DOWNLOADING << 8) | STATE_IDLE: /* Downloading -> idle */
	case (STATE_DOWNLOADED << 8) | STATE_IDLE:  /* Downloaded -> idle */
	case (STATE_UPDATING << 8) | STATE_IDLE:    /* Updating -> idle */
		ret = 0;
		break;
	default:
		break;
	}

	if (!ret) {
		path.res_id = ADV_FW_STATE_ID;

		ret = lwm2m_set_u8(&path, state);
		if (!ret) {
			adv_fw_update_last_state_chg_time[obj_inst_id] = time(NULL);
		}
	}
	lwm2m_registry_unlock();

	if (!ret) {
		LOG_DBG("Inst %u: state %u", (unsigned int)obj_inst_id, (unsigned int)state);
	} else {
		LOG_ERR("Error %d on state transition: %u -> %u", ret, (unsigned int)prev_state,
			(unsigned int)state);
	}
}

static void lwm2m_adv_fw_set_update_result_inst(uint16_t obj_inst_id, uint8_t result)
{
	int ret;
	bool error = false;
	uint8_t state, prev_state;
	struct lwm2m_obj_path path = LWM2M_OBJ(LWM2M_OBJECT_ADVANCED_FIRMWARE_UPDATE_ID,
					       obj_inst_id, ADV_FW_UPDATE_RESULT_ID);

	lwm2m_registry_lock();

	state = STATE_IDLE;
	prev_state = adv_fw_update_state[obj_inst_id];
	switch (result) {
	case RESULT_DEFAULT:
		lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_IDLE);
		break;
	case RESULT_SUCCESS:
		if (prev_state != STATE_UPDATING) {
			error = false;
			state = prev_state;
		}

		lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_IDLE);
		break;
	case RESULT_NO_STORAGE:
	case RESULT_OUT_OF_MEM:
	case RESULT_CONNECTION_LOST:
	case RESULT_UNSUP_FW:
	case RESULT_INVALID_URI:
	case RESULT_UNSUP_PROTO:
		if (prev_state != STATE_DOWNLOADING) {
			error = true;
			state = prev_state;
		}

		lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_IDLE);
		break;
	case RESULT_INTEGRITY_FAILED:
	case RESULT_UPDATE_FAILED:
		if (prev_state != STATE_DOWNLOADING && prev_state != STATE_UPDATING) {
			error = true;
			state = prev_state;
		}

		lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_IDLE);
		break;
	default:
		LOG_ERR("Unexpected result 0x%x", (unsigned int)result);
		lwm2m_registry_unlock();
		return;
	}

	if (error) {
		LOG_ERR("Result %u unexpected in state %u", (unsigned int)result,
			(unsigned int)state);
	}

	ret = lwm2m_set_u8(&path, result);
	lwm2m_registry_unlock();

	if (!ret) {
		LOG_DBG("Inst %u: state %u", (unsigned int)obj_inst_id, (unsigned int)state);
	} else {
		LOG_ERR("Could not update result: %d", ret);
	}
}

static int adv_fw_update_write_cb(uint16_t obj_inst_id, uint16_t res_id, uint16_t res_inst_id,
				  uint8_t *data, uint16_t data_len, bool last_block,
				  size_t total_size, size_t offset)
{
	int ret = 0;
	uint8_t state, result;
	lwm2m_engine_set_data_cb_t wr_cb;
	lwm2m_engine_user_cb_t cancel_cb;

	state = lwm2m_adv_fw_get_update_state_inst(obj_inst_id);
	switch (state) {
	case STATE_IDLE:
		lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_DOWNLOADING);
		break;
	case STATE_DOWNLOADED:
		if (!data_len || (data_len == 1u && *data == '\0')) {
			lwm2m_adv_fw_set_update_result_inst(obj_inst_id, RESULT_DEFAULT);
			cancel_cb = lwm2m_adv_fw_get_cancel_cb_inst(obj_inst_id);

			if (cancel_cb) {
				ret = cancel_cb(obj_inst_id);
			}

			LOG_DBG("Update cancelled by writing %u bytes", (unsigned int)data_len);
			return 0;
		}
		LOG_WRN("Download already completed");
		return -EPERM;
	case STATE_DOWNLOADING:
		break;
	default:
		LOG_WRN("Cannot download in state %u", (unsigned int)state);
		return -EPERM;
	}

	wr_cb = lwm2m_adv_fw_get_write_cb_inst(obj_inst_id);

	if (wr_cb) {
		ret = wr_cb(obj_inst_id, res_id, res_inst_id, data, data_len, last_block,
			    total_size, offset);
	}

	result = RESULT_UPDATE_FAILED;
	switch (ret) {
	case -ENOMEM:
		result = RESULT_OUT_OF_MEM;
		break;
	case -ENOSPC:
		result = RESULT_NO_STORAGE;
		ret = -EFBIG;
		break;
	case -EFAULT:
		result = RESULT_INTEGRITY_FAILED;
		break;
	case -ENOMSG:
		result = RESULT_UNSUP_FW;
		break;
	default:
		break;
	}

	if (ret >= 0) {
		if (last_block) {
			lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_DOWNLOADED);
		}
	} else {
		lwm2m_adv_fw_set_update_result_inst(obj_inst_id, result);
	}

	return ret;
}

static void lwm2m_adv_fw_set_update_result(uint16_t obj_inst_id, int ec)
{
	uint8_t result;

	if (!IS_ENABLED(CONFIG_LWM2M_ADVANCED_FIRMWARE_UPDATE_PULL_SUPPORT)) {
		return;
	}

	switch (ec) {
	case 0:
		lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_DOWNLOADED);
		return;
	case -ENOMEM:
		result = RESULT_OUT_OF_MEM;
		break;
	case -ENOSPC:
		result = RESULT_NO_STORAGE;
		break;
	case -EFAULT:
		result = RESULT_INTEGRITY_FAILED;
		break;
	case -ENOMSG:
		result = RESULT_CONNECTION_LOST;
		break;
	case -ENOTSUP:
		result = RESULT_INVALID_URI;
		break;
	case -EPROTONOSUPPORT:
		result = RESULT_UNSUP_PROTO;
		break;
	default:
		result = RESULT_UPDATE_FAILED;
		break;
	}

	lwm2m_adv_fw_set_update_result_inst(obj_inst_id, result);
}

static int lwm2m_adv_fw_start_transfer(uint16_t obj_inst_id, char *package_uri)
{
	struct requesting_object req = {
		.obj_inst_id = obj_inst_id,
		.is_firmware_uri = true,
		.result_cb = lwm2m_adv_fw_set_update_result,
		.write_cb = lwm2m_adv_fw_get_write_cb_inst(obj_inst_id),
		.verify_cb = NULL,
	};

	if (!IS_ENABLED(CONFIG_LWM2M_ADVANCED_FIRMWARE_UPDATE_PULL_SUPPORT)) {
		return -ENOTSUP;
	}

	if (unlikely(!req.write_cb)) {
		return -EINVAL;
	}

	return lwm2m_pull_context_start_transfer(package_uri, req, K_NO_WAIT);
}

static int adv_fw_update_pkg_uri_write_cb(uint16_t obj_inst_id, uint16_t res_id,
					  uint16_t res_inst_id, uint8_t *data, uint16_t data_len,
					  bool last_block, size_t total_size, size_t offset)
{
	uint8_t state;
	bool empty_uri;

	if (!IS_ENABLED(CONFIG_LWM2M_ADVANCED_FIRMWARE_UPDATE_PULL_SUPPORT)) {
		return -ENOTSUP;
	}

	if (unlikely(obj_inst_id >= ARRAY_SIZE(adv_fw_update_pkg_uri))) {
		return -EINVAL;
	}

	state = lwm2m_adv_fw_get_update_state_inst(obj_inst_id);
	empty_uri = !data_len || !*data;

	switch (state) {
	case STATE_IDLE:
		if (!empty_uri) {
			lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_DOWNLOADING);
			lwm2m_adv_fw_start_transfer(obj_inst_id,
						    adv_fw_update_pkg_uri[obj_inst_id]);
		}
		break;
	case STATE_DOWNLOADED:
		if (empty_uri) {
			/* Reset state */
			lwm2m_adv_fw_set_update_result_inst(obj_inst_id, RESULT_DEFAULT);
		}
		break;
	}

	return 0;
}

static int adv_fw_update_cb(uint16_t obj_inst_id, uint8_t *args, uint16_t args_len)
{
	int ret;
	uint8_t state, result;
	lwm2m_engine_execute_cb_t cb;

	state = lwm2m_adv_fw_get_update_state_inst(obj_inst_id);
	if (state != STATE_DOWNLOADED) {
		LOG_ERR("Unexpected state 0x%x", (unsigned int)state);
		return -EPERM;
	}

	lwm2m_adv_fw_set_update_state_inst(obj_inst_id, STATE_UPDATING);

	cb = lwm2m_adv_fw_get_update_cb_inst(obj_inst_id);
	if (cb) {
		ret = cb(obj_inst_id, args, args_len);

		if (ret < 0) {
			LOG_ERR("Error updating firmware: %d", ret);

			result = ret == -EINVAL ? RESULT_INTEGRITY_FAILED : RESULT_UPDATE_FAILED;
			lwm2m_adv_fw_set_update_result_inst(obj_inst_id, result);
			return 0;
		}
	}

	return 0;
}

static int adv_fw_cancel_cb(uint16_t obj_inst_id, uint8_t *args, uint16_t args_len)
{
	int ret;
	lwm2m_engine_user_cb_t cancel_cb;

	/* Reset state */
	lwm2m_adv_fw_set_update_result_inst(obj_inst_id, RESULT_DEFAULT);
	cancel_cb = lwm2m_adv_fw_get_cancel_cb_inst(obj_inst_id);

	/* TODO: cancel potential pull */

	ret = 0;
	if (cancel_cb) {
		ret = cancel_cb(obj_inst_id);
	}

	LOG_DBG("Update cancelled");
	return ret;
}

static struct lwm2m_engine_obj_inst *lwm2m_adv_fw_update_create_obj_inst(uint16_t obj_inst_id)
{
	int i = 0, j = 0;
	unsigned int index = ARRAY_SIZE(inst);

	for (unsigned int ii = 0u; ii < ARRAY_SIZE(inst); ++ii) {
		if (!inst[ii].obj) {
			index = index == ARRAY_SIZE(inst) ? ii : index;
		} else if (inst[ii].obj_inst_id == obj_inst_id) {
			LOG_ERR("Cannot create object instance %u, it already exists", obj_inst_id);
			return NULL;
		}
	}

	if (index == ARRAY_SIZE(inst)) {
		LOG_ERR("All instances exhausted, cannot create %u", obj_inst_id);
		return NULL;
	}

	init_res_instance(res_inst[index], ARRAY_SIZE(res_inst[index]));

	INIT_OBJ_RES_OPT(ADV_FW_PKG_ID, res[index], i, res_inst[index], j, 1, false, true, NULL,
			 NULL, NULL, adv_fw_update_write_cb, NULL);
	INIT_OBJ_RES_LEN(ADV_FW_PKG_URI_ID, res[index], i, res_inst[index], j, 1, false, true,
			 adv_fw_update_pkg_uri[index], LWM2M_PACKAGE_URI_LEN, 0, NULL, NULL, NULL,
			 adv_fw_update_pkg_uri_write_cb, NULL);
	INIT_OBJ_RES_EXECUTE(ADV_FW_UPDATE_ID, res[index], i, adv_fw_update_cb);
	INIT_OBJ_RES_DATA(ADV_FW_STATE_ID, res[index], i, res_inst[index], j,
			  &adv_fw_update_state[index], sizeof(adv_fw_update_state[index]));
	INIT_OBJ_RES_DATA(ADV_FW_UPDATE_RESULT_ID, res[index], i, res_inst[index], j,
			  &adv_fw_update_result[index], sizeof(adv_fw_update_result[index]));
	INIT_OBJ_RES_OPTDATA(ADV_FW_PKG_NAME_ID, res[index], i, res_inst[index], j);
	INIT_OBJ_RES_OPTDATA(ADV_FW_PKG_VERSION_ID, res[index], i, res_inst[index], j);
	INIT_OBJ_RES_MULTI_OPTDATA(ADV_FW_UPDATE_PROTO_SUPPORT_ID, res[index], i, res_inst[index],
				   j, 1, false);
	INIT_OBJ_RES_DATA(ADV_FW_DELIVERY_METHOD_ID, res[index], i, res_inst[index], j,
			  &adv_fw_update_delivery_method[index],
			  sizeof(adv_fw_update_delivery_method[index]));
	INIT_OBJ_RES_EXECUTE(ADV_FW_CANCEL_ID, res[index], i, adv_fw_cancel_cb);
	INIT_OBJ_RES_DATA(ADV_FW_SEVERITY_ID, res[index], i, res_inst[index], j,
			  &adv_fw_update_severity[index], sizeof(adv_fw_update_severity[index]));
	INIT_OBJ_RES_DATA(ADV_FW_LAST_STATE_CHANGE_TIME_ID, res[index], i, res_inst[index], j,
			  &adv_fw_update_last_state_chg_time[index],
			  sizeof(adv_fw_update_last_state_chg_time[index]));
	INIT_OBJ_RES_DATA(ADV_FW_MAX_DEFER_PERIOD_ID, res[index], i, res_inst[index], j,
			  &adv_fw_update_max_defer_period[index],
			  sizeof(adv_fw_update_max_defer_period[index]));
	INIT_OBJ_RES_OPTDATA(ADV_FW_COMPONENT_NAME_ID, res[index], i, res_inst[index], j);
	INIT_OBJ_RES_OPTDATA(ADV_FW_CURRENT_VERSION_ID, res[index], i, res_inst[index], j);

	inst[index].resources = res[index];
	inst[index].resource_count = i;

	LOG_DBG("Create LwM2M advanced firmware instance %u", (unsigned int)obj_inst_id);
	return &inst[index];
}

static int lwm2m_adv_fw_update_init(void)
{
	int ret;
	struct lwm2m_engine_obj_inst *obj_inst = NULL;

	adv_fw_update.obj_id = LWM2M_OBJECT_ADVANCED_FIRMWARE_UPDATE_ID;
	adv_fw_update.version_major = ADV_FW_VERSION_MAJOR;
	adv_fw_update.version_minor = ADV_FW_VERSION_MINOR;
	/* Relatively universally accepted but not part of the specification */
	adv_fw_update.is_core = false;
	adv_fw_update.fields = fields;
	adv_fw_update.field_count = ARRAY_SIZE(fields);
	adv_fw_update.max_instance_count = ARRAY_SIZE(inst);
	adv_fw_update.create_cb = lwm2m_adv_fw_update_create_obj_inst;
	lwm2m_register_obj(&adv_fw_update);

	ret = 0;
	for (unsigned int i = 0u; !ret && i < adv_fw_update.max_instance_count; ++i) {
		adv_fw_update_state[i] = STATE_IDLE;
		adv_fw_update_result[i] = RESULT_DEFAULT;
		/* Default severity is 1 */
		adv_fw_update_severity[i] = LWM2M_SEVERITY_MANDATORY;
		adv_fw_update_last_state_chg_time[i] = time(NULL);

		BUILD_ASSERT(ADV_FW_DELIVERY_BOTH == ADV_FW_DELIVERY_PUSH_ONLY + 1);
		adv_fw_update_delivery_method[i] =
			ADV_FW_DELIVERY_PUSH_ONLY +
			IS_ENABLED(CONFIG_LWM2M_ADVANCED_FIRMWARE_UPDATE_PULL_SUPPORT);

		LOG_DBG("Creating %d instance %u", LWM2M_OBJECT_ADVANCED_FIRMWARE_UPDATE_ID, i);
		ret = lwm2m_create_obj_inst(LWM2M_OBJECT_ADVANCED_FIRMWARE_UPDATE_ID, i, &obj_inst);
		if (ret < 0) {
			LOG_ERR("Could not create LwM2M %d instance %u: %d",
				LWM2M_OBJECT_ADVANCED_FIRMWARE_UPDATE_ID, i, ret);
		}
	}

	return ret;
}

LWM2M_CORE_INIT(lwm2m_adv_fw_update_init);

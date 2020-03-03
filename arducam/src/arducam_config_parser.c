#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "arducam/arducam_config_parser.h"
#include "arducam/ini.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

TypeMap section_types[] = {
    {"camera parameter", SECTION_TYPE_CAMERA},
    {"board parameter", SECTION_TYPE_BOARD},
    {"board parameter||dev2", SECTION_TYPE_BOARD_2},
    {"board parameter||dev3||inf2", SECTION_TYPE_BOARD_3_2},
    {"board parameter||dev3||inf3", SECTION_TYPE_BOARD_3_3},
    {"register parameter", SECTION_TYPE_REG},
    {"register parameter||dev3||inf2", SECTION_TYPE_REG_3_2},
    {"register parameter||dev3||inf3", SECTION_TYPE_REG_3_3},
    {0, 0},
};

TypeMap config_types[] = {
    {"REG", CONFIG_TYPE_REG},
    {"DELAY", CONFIG_TYPE_DELAY},
    {"VRCMD", CONFIG_TYPE_VRCMD},
    {0, 0},
};

static uint32_t get_type(TypeMap map[], const char* name) {
    int i;
    for (i = 0; map[i].name && strcmp(map[i].name, name); i++)
        ;
    return map[i].type;
}

static uint32_t parse_number(const char* value) {
    const char* temp = value;
    for (int i = 0; i < strlen(value); i++) {
        if (isspace((unsigned char)*temp)) {
            temp++;
            continue;
        }
    }
    if (*temp == '0' && *(temp + 1) == 'x')
        return strtol(value, NULL, 16);
    else
        return strtol(value, NULL, 10);
}

static void parse_params(Config* config, const char* src) {
    const char* delim  = ",";
    int         count  = 0;
    char*       origin = (char*)malloc(strlen(src) + 1);
    strncpy(origin, src, strlen(src) + 1);
    char* result;
    result = strtok(origin, delim);
    while (result != NULL) {
        config->params[count++] = parse_number(result);
        result                  = strtok(NULL, delim);
    }
    config->params_length = count;
    free(origin);
}

static void parse_camera_parameter(CameraParam* camera_param, const char* name, const char* value) {
    Config cfg;
    if (!strcmp(name, "CFG_MODE")) {
        camera_param->cfg_mode = parse_number(value);
    } else if (!strcmp(name, "TYPE")) {
        strncpy(camera_param->type, value, sizeof(camera_param->type));
    } else if (!strcmp(name, "BIT_WIDTH")) {
        camera_param->bit_width = parse_number(value);
    } else if (!strcmp(name, "I2C_MODE")) {
        camera_param->i2c_mode = parse_number(value);
    } else if (!strcmp(name, "I2C_ADDR")) {
        camera_param->i2c_addr = parse_number(value);
    } else if (!strcmp(name, "TRANS_LVL")) {
        camera_param->trans_lvl = parse_number(value);
    } else if (!strcmp(name, "SIZE")) {
        parse_params(&cfg, value);
        if (cfg.params_length == 2) {
            camera_param->width  = cfg.params[0];
            camera_param->height = cfg.params[1];
        }
    } else if (!strcmp(name, "FORMAT")) {
        parse_params(&cfg, value);
        if (cfg.params_length == 2)
            camera_param->format = (cfg.params[0] << 8) | cfg.params[1];
        else if (cfg.params_length == 1)
            camera_param->format = (cfg.params[0] << 8);
    }
}

static int parser_handle(void* user, const char* section, const char* name, const char* value) {
    if (!user)
        return 0;
    CameraConfigs* configs = (CameraConfigs*)user;
    uint32_t       config_type, section_type;

    if (!(section_type = get_type(section_types, section)))
        return 1;

    if (section_type == SECTION_TYPE_CAMERA) {
        parse_camera_parameter(&configs->camera_param, name, value);
    } else if ((config_type = get_type(config_types, name))) {
        Config* config = &configs->configs[configs->configs_length++];
        config->type   = section_type | config_type;
        parse_params(config, value);
    }

    return 1;
}

void dump_camera_parameter(CameraParam* camera_param) {
    LOG("CFG_MODE : %d", camera_param->cfg_mode);
    LOG("TYPE     : %s", camera_param->type);
    LOG("WIDTH    : %d", camera_param->width);
    LOG("HEIGHT   : %d", camera_param->height);
    LOG("BIT_WIDTH: %d", camera_param->bit_width);
    LOG("FORMAT   : 0x%02X", camera_param->format);
    LOG("I2C_MODE : %d", camera_param->i2c_mode);
    LOG("I2C_ADDR : %d", camera_param->i2c_addr);
    LOG("TRANS_LVL: %d", camera_param->trans_lvl);
}

void dump_configs(CameraConfigs* configs) {
    uint32_t lastSection = 0;
    for (int i = 0; i < configs->configs_length; i++) {
        if (lastSection != (configs->configs[i].type >> 16)) {
            LOG("[0x%04X]", (configs->configs[i].type >> 16));
            lastSection = configs->configs[i].type >> 16;
        }
        for (int j = 0; j < configs->configs[i].params_length; j++)
            printf("0x%04X ", configs->configs[i].params[j]);
        printf("\n");
    }
}

int arducam_parse_config(const char* file_name, CameraConfigs* cam_cfgs) {
    return ini_parse(file_name, parser_handle, cam_cfgs);
}
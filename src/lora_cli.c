/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#if (LORA_NODE_CLI) || (LORA_NODE_LOG_CLI)
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "node/lora_priv.h"
#endif

#if (LORA_NODE_CLI)

static int lora_cli_cmd_fn(int argc, char **argv);
static int lora_cli_set_freq(int argc, char **argv);
static int lora_cli_tx_cfg(int argc, char **argv);
static int lora_cli_rx_cfg(int argc, char **argv);
static int lora_cli_tx(int argc, char **argv);
static int lora_cli_rx(int argc, char **argv);
static int lora_cli_max_payload_len(int argc, char **argv);

//// #warning Convert to BL602 CLI

/**
 * Parses a long long within an imposed range.
 *
 * @param sval                  The string to parse.
 * @param min                   Values less than this are rejected.
 * @param max                   Values greater than this are rejected.
 * @param out_status            Written on completion;
 *                                  0: success;
 *                                  SYS_EINVAL: invalid string or number out of
 *                                      range.
 *
 * @return                      The parsed number on success;
 *                              unspecified on error.
 */
long long
parse_ll_bounds(const char *sval, long long min, long long max,
                int *out_status);

/**
 * Parses an unsigned long long within an imposed range.
 *
 * @param sval                  The string to parse.
 * @param min                   Values less than this are rejected.
 * @param max                   Values greater than this are rejected.
 * @param out_status            Written on completion;
 *                                  0: success;
 *                                  SYS_EINVAL: invalid string or number out of
 *                                      range.
 *
 * @return                      The parsed number on success;
 *                              unspecified on error.
 */
unsigned long long
parse_ull_bounds(const char *sval,
                 unsigned long long min, unsigned long long max,
                 int *out_status);

/**
 * Parses an unsigned long long.
 *
 * @param sval                  The string to parse.
 * @param out_status            Written on completion;
 *                                  0: success;
 *                                  SYS_EINVAL: invalid string or number out of
 *                                      range.
 *
 * @return                      The parsed number on success;
 *                              unspecified on error.
 */
unsigned long long
parse_ull(const char *sval, int *out_status);

/**
 * @brief Parses a stream of bytes using ':' or '-' as delimiters.
 *
 * Parses a stream of bytes using ':' or '-' as delimiters.
 * The base of each byte string is inferred from the text (base 16 if prefixed
 * with "0x", base 10 otherwise).
 *
 * @param sval                  The string to parse.
 * @param max_len               The maximum number of bytes to write.
 * @param dst                   The destination buffer to write bytes to.
 * @param out_len               Written on success; total number of bytes
 *                                  written to the destination buffer.
 *
 * @return                      0 on success;
 *                              SYS_EINVAL on invalid byte stream;
 *                              SYS_ERANGE if result only partially written to
 *                                  buffer due to insufficient space.
 */
int
parse_byte_stream(const char *sval, int max_len, uint8_t *dst, int *out_len);

#ifdef TODO
static struct shell_cmd lora_cli_cmd = {
    .sc_cmd = "lora",
    .sc_cmd_func = lora_cli_cmd_fn,
};

static struct shell_cmd lora_cli_subcmds[] = {
    {
        .sc_cmd = "set_freq",
        .sc_cmd_func = lora_cli_set_freq,
    },
    {
        .sc_cmd = "tx_cfg",
        .sc_cmd_func = lora_cli_tx_cfg,
    },
    {
        .sc_cmd = "rx_cfg",
        .sc_cmd_func = lora_cli_rx_cfg,
    },
    {
        .sc_cmd = "tx",
        .sc_cmd_func = lora_cli_tx,
    },
    {
        .sc_cmd = "rx",
        .sc_cmd_func = lora_cli_rx,
    },
    {
        .sc_cmd = "max_payload_len",
        .sc_cmd_func = lora_cli_max_payload_len,
    },
};

static int
lora_cli_cmd_fn(int argc, char **argv)
{
    const struct shell_cmd *subcmd;
    const char *err;
    int rc;
    int i;

    if (argc <= 1) {
        rc = 1;
        err = NULL;
        goto err;
    }

    for (i = 0;
         i < sizeof lora_cli_subcmds / sizeof lora_cli_subcmds[0];
         i++) {

        subcmd = lora_cli_subcmds + i;
        if (strcmp(argv[1], subcmd->sc_cmd) == 0) {
            rc = subcmd->sc_cmd_func(argc - 1, argv + 1);
            return rc;
        }
    }

    rc = 1;
    err = "invalid lora command";

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora set_freq\r\n"
"    lora tx_cfg\r\n"
"    lora rx_cfg\r\n"
"    lora tx\r\n"
"    lora rx\r\n"
"    lora max_payload_len\r\n");

    return rc;
}
#endif  //  TODO

static int
lora_cli_set_freq(int argc, char **argv)
{
    const char *err;
    uint32_t freq;
    int rc;

    if (argc <= 1) {
        rc = 1;
        err = NULL;
        goto err;
    }

    freq = parse_ull(argv[1], &rc);
    if (rc != 0) {
        err = "invalid frequency";
        goto err;
    }

    Radio.SetChannel(freq);
    return 0;

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora set_freq <hz>\r\n");

    return rc;
}

static int
lora_cli_tx_cfg(int argc, char **argv)
{
    RadioModems_t modem;
    const char *err;
    char **arg;
    uint32_t bandwidth;
    uint32_t datarate;
    uint32_t timeout;
    uint32_t fdev;
    uint16_t preamble_len;
    uint8_t hop_period;
    uint8_t coderate;
    int8_t power;
    int freq_hop_on;
    int iq_inverted;
    int fix_len;
    int crc_on;
    int rc;

    if (argc <= 13) {
        rc = 1;
        err = NULL;
        goto err;
    }

    arg = argv + 1;

    modem = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    power = parse_ll_bounds(*arg, INT8_MIN, INT8_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    fdev = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    bandwidth = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    datarate = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    coderate = parse_ull_bounds(*arg, 0, UINT8_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    preamble_len = parse_ull_bounds(*arg, 0, UINT16_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    fix_len = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    crc_on = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    freq_hop_on = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    hop_period = parse_ull_bounds(*arg, 0, UINT8_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    iq_inverted = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    timeout = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        goto err;
    }
    arg++;

    Radio.SetTxConfig(modem,
                      power,
                      fdev,
                      bandwidth,
                      datarate,
                      coderate,
                      preamble_len,
                      fix_len,
                      crc_on,
                      freq_hop_on,
                      hop_period,
                      iq_inverted,
                      timeout);

    return 0;

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora tx_cfg <modem-type (0/1)> <power> <frequency-deviation>\r\n"
"                <bandwidth> <data-rate> <code-rate> <preamble-length>\r\n"
"                <fixed-length (0/1)> <crc-on (0/1)>\r\n"
"                <frequency-hopping (0/1)> <hop-period> <iq-inverted (0/1)>\r\n"
"                <timeout>\r\n");

    return rc;
}

static int
lora_cli_rx_cfg(int argc, char **argv)
{
    RadioModems_t modem;
    const char *err;
    char **arg;
    uint32_t bandwidth_afc;
    uint32_t bandwidth;
    uint32_t datarate;
    uint16_t preamble_len;
    uint16_t symb_timeout;
    uint8_t payload_len;
    uint8_t hop_period;
    uint8_t coderate;
    int rx_continuous;
    int freq_hop_on;
    int iq_inverted;
    int fix_len;
    int crc_on;
    int rc;

    if (argc <= 14) {
        rc = 1;
        err = NULL;
        goto err;
    }

    arg = argv + 1;

    modem = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        err = "invalid modem type";
        goto err;
    }
    arg++;

    bandwidth = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        err = "invalid bandwidth";
        goto err;
    }
    arg++;

    datarate = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        err = "invalid data rate";
        goto err;
    }
    arg++;

    coderate = parse_ull_bounds(*arg, 0, UINT8_MAX, &rc);
    if (rc != 0) {
        err = "invalid code rate";
        goto err;
    }
    arg++;

    bandwidth_afc = parse_ull_bounds(*arg, 0, UINT32_MAX, &rc);
    if (rc != 0) {
        err = "invalid bandwidtch_afc";
        goto err;
    }
    arg++;

    preamble_len = parse_ull_bounds(*arg, 0, UINT16_MAX, &rc);
    if (rc != 0) {
        err = "invalid preamble length";
        goto err;
    }
    arg++;

    symb_timeout = parse_ull_bounds(*arg, 0, UINT16_MAX, &rc);
    if (rc != 0) {
        err = "invalid symbol timeout";
        goto err;
    }
    arg++;

    fix_len = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        err = "invalid fixed length value";
        goto err;
    }
    arg++;

    payload_len = parse_ull_bounds(*arg, 0, UINT8_MAX, &rc);
    if (rc != 0) {
        err = "invalid payload length";
        goto err;
    }
    arg++;

    crc_on = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        err = "invalid crc on value";
        goto err;
    }
    arg++;

    freq_hop_on = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        err = "invalid frequency hopping value";
        goto err;
    }
    arg++;

    hop_period = parse_ull_bounds(*arg, 0, UINT8_MAX, &rc);
    if (rc != 0) {
        err = "invalid hop period";
        goto err;
    }
    arg++;

    iq_inverted = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        err = "invalid iq inverted value";
        goto err;
    }
    arg++;

    rx_continuous = parse_ull_bounds(*arg, 0, 1, &rc);
    if (rc != 0) {
        err = "invalid rx continuous value";
        goto err;
    }
    arg++;

    Radio.SetRxConfig(modem,
                      bandwidth,
                      datarate,
                      coderate,
                      bandwidth_afc,
                      preamble_len,
                      symb_timeout,
                      fix_len,
                      payload_len,
                      crc_on,
                      freq_hop_on,
                      hop_period,
                      iq_inverted,
                      rx_continuous);

    return 0;

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora rx_cfg <modem-type (0/1)> <bandwidth> <data-rate> <code-rate>\r\n"
"                <bandwidtch-afc> <preamble-length> <symbol-timeout>\r\n"
"                <fixed-length (0/1)> <payload-length> <crc-on (0/1)>\r\n"
"                <frequency-hopping (0/1)> <hop-period> <iq-inverted (0/1)>\r\n"
"                <rx-continuous (0/1)>\r\n");

    return rc;
}

static int
lora_cli_tx(int argc, char **argv)
{
    uint8_t buf[UINT8_MAX];
    const char *err;
    int buf_sz;
    int rc;

    if (argc <= 1) {
        rc = 1;
        err = NULL;
        goto err;
    }

    rc = parse_byte_stream(argv[1], sizeof buf, buf, &buf_sz);
    if (rc != 0) {
        err = "invalid payload";
        goto err;
    }

    Radio.Send(buf, buf_sz);
    return 0;

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora tx <0xXX:0xXX:...>\r\n");

    return rc;
}

static int
lora_cli_rx(int argc, char **argv)
{
    const char *err;
    uint32_t timeout;
    int rc;

    if (argc <= 1) {
        rc = 1;
        err = NULL;
        goto err;
    }

    timeout = parse_ull_bounds(argv[1], 0, UINT32_MAX, &rc);
    if (rc != 0) {
        err = "invalid timeout";
        goto err;
    }

    Radio.Rx(timeout);
    return 0;

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora rx <timeout>\r\n");

    return rc;
}

static int
lora_cli_max_payload_len(int argc, char **argv)
{
    RadioModems_t modem;
    const char *err;
    uint8_t len;
    int rc;

    if (argc <= 2) {
        rc = 1;
        err = NULL;
        goto err;
    }

    modem = parse_ull_bounds(argv[1], 0, 1, &rc);
    if (rc != 0) {
        err = "invalid modem type";
        goto err;
    }

    len = parse_ull_bounds(argv[2], 0, UINT8_MAX, &rc);
    if (rc != 0) {
        err = "invalid length";
        goto err;
    }

    Radio.SetMaxPayloadLength(modem, len);
    return 0;

err:
    if (err != NULL) {
        printf("error: %s\r\n", err);
    }

    printf(
"usage:\r\n"
"    lora max_payload_len <length>\r\n");

    return rc;
}

#endif /* (LORA_NODE_CLI) */

#if (LORA_NODE_LOG_CLI) == 1

#if (LORA_NODE_LOG_CLI) == 1
int lora_cli_log_cmd(int argc, char **argv);

#ifdef TODO  //  Convert to BL602 CLI
static struct shell_cmd lora_node_log_cmd = {
    .sc_cmd = "ln_log",
    .sc_cmd_func = lora_cli_log_cmd
};
#endif  //  TODO

#endif

//// #warning Convert lora_cli_log_cmd to BL602 CLI
int
lora_cli_log_cmd(int argc, char **argv)
{
#ifdef LORA_NODE_DEBUG_LOG
    uint16_t i;
    uint16_t lines_logged;

    printf("Lora node log\r\n");
    i = g_lnd_log_index;
    lines_logged = 0;
    while (lines_logged != LORA_NODE_DEBUG_LOG_ENTRIES) {
        /* Do not display empty log lines */
        if (g_lnd_log[i].lnd_id == 0) {
            goto next_entry;
        }

        printf("index=%u ", i);
        switch (g_lnd_log[i].lnd_id) {
        case LORA_NODE_LOG_TX_DONE:
            printf("TX_DONE chan=%u done_time=%lu",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_TX_SETUP:
            printf("TX_SETUP phytxpwr=%d sf=%u bw=%u freq=%lu",
                           (int8_t)g_lnd_log[i].lnd_p8,
                           (uint8_t)(g_lnd_log[i].lnd_p16 >> 8),
                           (uint8_t)g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_TX_START:
            printf("TX_START pwr=%d dr=%u chan=%u airtime=%lu",
                           (int8_t)g_lnd_log[i].lnd_p8,
                           (uint8_t)(g_lnd_log[i].lnd_p16 >> 8),
                           (uint8_t)g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_TX_DELAY:
            printf("TX_DELAY dc=%u delay_usecs=%lu",
                           (int8_t)g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p32);
            break;
       case LORA_NODE_LOG_TX_PREP_FRAME:
            printf("TX_PREP_FRAME cmdbytes=%u uplink=%u mhdr=%x",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           (uint8_t)g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_RX_WIN1_SETUP:
            printf("RX_WIN1_SETUP dr=%u chan=%u timeout=%lu",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_RX_TIMEOUT:
            printf("RX_TIMEOUT chan=%u rxslot=%u",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16);
            break;
        case LORA_NODE_LOG_RX_DONE:
            printf("RX_DONE chan=%u size=%u slot=%u machdr=%x",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           (uint8_t)(g_lnd_log[i].lnd_p32 >> 8),
                           (uint8_t)g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_RADIO_TIMEOUT_IRQ:
            break;
        case LORA_NODE_LOG_RX_CFG:
            printf("RX_CFG bw=%u dr=%u sf=%u freq=%lu",
                           (int8_t)g_lnd_log[i].lnd_p8,
                           (uint8_t)(g_lnd_log[i].lnd_p16 >> 8),
                           (uint8_t)g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_RX_PORT:
            printf("RX_PORT port=%u len=%u dwnlink_cntr=%lu",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_RX_WIN2:
            printf("RX_WIN2 rxslot=%u cont=%u freq=%lu",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_APP_TX:
            printf("APP_TX pktlen=%u om=%lx",
                           g_lnd_log[i].lnd_p16, g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_RTX_TIMEOUT:
            printf("RTX_TIMEOUT macflags=%x", g_lnd_log[i].lnd_p8);
            break;
        case LORA_NODE_LOG_RX_ADR_REQ:
            printf("RX_ADR_REQ dr=%u txpwr=%u chmassk=%u nbrep=%u",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           (uint16_t)(g_lnd_log[i].lnd_p32 >> 16),
                           (uint16_t)g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_PROC_MAC_CMD:
            printf("PROC_MAC_CMD index=%u snr=%u cmd_size=%lu",
                           g_lnd_log[i].lnd_p8, g_lnd_log[i].lnd_p16,
                           g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_LINK_CHK:
            printf("LINK_CHK status=%lu", g_lnd_log[i].lnd_p32);
            break;
        case LORA_NODE_LOG_LP_ENTER:
            printf("LOW POWER ENTER");
            break;
        case LORA_NODE_LOG_LP_EXIT:
            printf("LOW POWER EXIT");
            break;
        case LORA_NODE_LOG_RX_WIN2_TIMEOUT:
            printf("RX_WIN2 TIMEOUT");
            break;
        case LORA_NODE_LOG_RX_WIN2_CANCEL:
            printf("RX_WIN2 CANCEL");
            break;
        default:
            printf("id=%u p8=%u p16=%u p32=%lu",
                           g_lnd_log[i].lnd_id, g_lnd_log[i].lnd_p8,
                           g_lnd_log[i].lnd_p16, g_lnd_log[i].lnd_p32);
            break;
        }

        printf(" cputime=%lu\r\n", g_lnd_log[i].lnd_cputime);

next_entry:
        ++i;
        if (i == LORA_NODE_DEBUG_LOG_ENTRIES) {
            i = 0;
        }
        ++lines_logged;
    }
#else
    printf("No Lora node log available\r\n");
#endif
    return 0;
}

#endif /* (LORA_NODE_LOG_CLI) */

void
lora_cli_init(void)
{
    int rc;

    (void)rc;
#if (LORA_NODE_CLI)

#ifdef TODO  //  Convert to BL602 CLI
    rc = shell_cmd_register(&lora_cli_cmd);
    SYSINIT_PANIC_ASSERT_MSG(rc == 0, "Failed to register lora CLI command");
#endif  //  TODO

#endif

#if (LORA_NODE_LOG_CLI)

#ifdef TODO  //  Convert to BL602 CLI
    rc = shell_cmd_register(&lora_node_log_cmd);
    assert(rc == 0);
#endif  //  TODO

#endif /* (LORA_NODE_LOG_CLI) */
}


// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_log.h"
#include "esp_eth.h"
#include "eth_phy/phy_reg.h"
#include "eth_phy/phy_lan8742.h"

#define LAN8742_PHY_ID1 0x0007
#define LAN8742_PHY_ID2 0xc130
#define LAN8742_PHY_ID2_MASK 0xFFF0

/* LAN8742-specific registers */
#define PHY_SPECIAL_CONTROL_STATUS_REG (0x1f)
#define AUTO_NEGOTIATION_DONE BIT(12)
#define DUPLEX_INDICATION_FULL BIT(4)
#define SPEED_INDICATION_100T BIT(3)
#define SPEED_INDICATION_10T BIT(2)
#define SPEED_DUPLEX_INDICATION_10T_HALF 0x04
#define SPEED_DUPLEX_INDICATION_10T_FULL 0x14
#define SPEED_DUPLEX_INDICATION_100T_HALF 0x08
#define SPEED_DUPLEX_INDICATION_100T_FULL 0x18

#define MMD_ACCESS_CTL_REG                 (13)
#define MMD_FUNCTION_ADDR                  0x00
#define MMD_FUNCTION_DATA                  BIT(14)
#define MMD_DEVAD_PCS                      3

#define MMD_ACCESS_ADDR_DATA_REG           (14)
#define PCS_MAC_RX_ADDRA_REG               (32865)
#define PCS_MAC_RX_ADDRB_REG               (32866)
#define PCS_MAC_RX_ADDRC_REG               (32867)

#define PCS_WUCSR                          (32784)
#define WOL_CONFIGURED                     BIT(8)
#define MPEN                               BIT(1)
#define MPR                                BIT(5)

#define ISFR                               (29)
#define ISFR_WOL_EVENT                     BIT(8)

#define IMR                                (30)

static const char *TAG = "lan8742";

// The device MMD registers adhere to the IEEE 802.3-2008 45.2 MDIO Interface
// Registers specification.
void esp_eth_mmd_write(uint16_t devad, uint16_t index, uint16_t value)
{
    esp_eth_smi_write(MMD_ACCESS_CTL_REG, MMD_FUNCTION_ADDR|devad);
    esp_eth_smi_write(MMD_ACCESS_ADDR_DATA_REG, index);
    esp_eth_smi_write(MMD_ACCESS_CTL_REG, MMD_FUNCTION_DATA|devad);
    esp_eth_smi_write(MMD_ACCESS_ADDR_DATA_REG, value);
}

uint16_t phy_lan8742_read_mmd_register(uint16_t devad, uint16_t index)
{
    esp_eth_smi_write(MMD_ACCESS_CTL_REG, MMD_FUNCTION_ADDR|devad);
    esp_eth_smi_write(MMD_ACCESS_ADDR_DATA_REG, index);
    esp_eth_smi_write(MMD_ACCESS_CTL_REG, MMD_FUNCTION_DATA|devad);
    return esp_eth_smi_read(MMD_ACCESS_ADDR_DATA_REG);
}


void phy_lan8742_check_phy_init(void)
{
    phy_lan8742_dump_registers();

    esp_eth_smi_wait_set(MII_BASIC_MODE_STATUS_REG, MII_AUTO_NEGOTIATION_COMPLETE, 0);
    esp_eth_smi_wait_set(PHY_SPECIAL_CONTROL_STATUS_REG, AUTO_NEGOTIATION_DONE, 0);
}

eth_speed_mode_t phy_lan8742_get_speed_mode(void)
{
    if (esp_eth_smi_read(PHY_SPECIAL_CONTROL_STATUS_REG) & SPEED_INDICATION_100T) {
        ESP_LOGD(TAG, "phy_lan8742_get_speed_mode(100)");
        return ETH_SPEED_MODE_100M;
    } else {
        ESP_LOGD(TAG, "phy_lan8742_get_speed_mode(10)");
        return ETH_SPEED_MODE_10M;
    }
}

eth_duplex_mode_t phy_lan8742_get_duplex_mode(void)
{
    if (esp_eth_smi_read(PHY_SPECIAL_CONTROL_STATUS_REG) & DUPLEX_INDICATION_FULL) {
        ESP_LOGD(TAG, "phy_lan8742_get_duplex_mode(FULL)");
        return ETH_MODE_FULLDUPLEX;
    } else {
        ESP_LOGD(TAG, "phy_lan8742_get_duplex_mode(HALF)");
        return ETH_MODE_HALFDUPLEX;
    }
}

void phy_lan8742_power_enable(bool enable)
{
    if (enable) {
        uint32_t data = esp_eth_smi_read(MII_BASIC_MODE_CONTROL_REG);
        data |= MII_AUTO_NEGOTIATION_ENABLE | MII_RESTART_AUTO_NEGOTIATION;
        esp_eth_smi_write(MII_BASIC_MODE_CONTROL_REG, data);
        // TODO: only enable if config.flow_ctrl_enable == true
        phy_mii_enable_flow_ctrl();
    }
}

esp_err_t phy_lan8742_init(void)
{
    ESP_LOGD(TAG, "phy_lan8742_init()");
    phy_lan8742_dump_registers();

    esp_eth_smi_write(MII_BASIC_MODE_CONTROL_REG, MII_SOFTWARE_RESET);

    esp_err_t res1, res2;
    // Call esp_eth_smi_wait_value() with a timeout so it prints an error periodically
    res1 = esp_eth_smi_wait_value(MII_PHY_IDENTIFIER_1_REG, LAN8742_PHY_ID1, UINT16_MAX, 1000);
    res2 = esp_eth_smi_wait_value(MII_PHY_IDENTIFIER_2_REG, LAN8742_PHY_ID2, LAN8742_PHY_ID2_MASK, 1000);

    uint32_t data = esp_eth_smi_read(MII_BASIC_MODE_CONTROL_REG);
    data |= MII_AUTO_NEGOTIATION_ENABLE | MII_RESTART_AUTO_NEGOTIATION;
    esp_eth_smi_write(MII_BASIC_MODE_CONTROL_REG, data);

    ets_delay_us(300);

    // TODO: only enable if config.flow_ctrl_enable == true
    phy_mii_enable_flow_ctrl();

    if (res1 == ESP_OK && res2 == ESP_OK) {
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

// Wake-On-LAN magic packet enable function.
void phy_lan8742_wol_magic_begin(void)
{
    uint8_t mac[6];
    esp_eth_get_mac(mac);
    // Tell the PHY which MAC pattern to watch for.
    esp_eth_mmd_write(MMD_DEVAD_PCS, PCS_MAC_RX_ADDRA_REG, (mac[5]<<8)|mac[4]);
    esp_eth_mmd_write(MMD_DEVAD_PCS, PCS_MAC_RX_ADDRB_REG, (mac[3]<<8)|mac[2]);
    esp_eth_mmd_write(MMD_DEVAD_PCS, PCS_MAC_RX_ADDRC_REG, (mac[1]<<8)|mac[0]);
    // Configure WoL.
    esp_eth_mmd_write(MMD_DEVAD_PCS, PCS_WUCSR, WOL_CONFIGURED|MPEN);
    // Enable the interrupt.
    esp_eth_smi_write(IMR, ISFR_WOL_EVENT);
}

// Wake-On-LAN magic packet disable function.
void phy_lan8742_wol_magic_end(void)
{
    // Deconfigure WoL.
    esp_eth_mmd_write(MMD_DEVAD_PCS, PCS_WUCSR, 0x00);
    // Read ISFR register 29 to clear any asserted WoL event on nINT.
    esp_eth_smi_write(IMR, ISFR_WOL_EVENT);
    esp_eth_smi_read(ISFR); // Bit#8 would be set for WoL event.
    // Mask interrupts.
    esp_eth_smi_write(IMR, 0);
    // Clear the magic packet received flag.
    esp_eth_mmd_write(MMD_DEVAD_PCS, PCS_WUCSR, MPR);
}

const eth_config_t phy_lan8742_default_ethernet_config = {
    .phy_addr = 0,
    .mac_mode = ETH_MODE_RMII,
    .clock_mode = ETH_CLOCK_GPIO0_IN,
    .flow_ctrl_enable = true,
    .phy_init = phy_lan8742_init,
    .phy_check_init = phy_lan8742_check_phy_init,
    .phy_power_enable = phy_lan8742_power_enable,
    .phy_check_link = phy_mii_check_link_status,
    .phy_get_speed_mode = phy_lan8742_get_speed_mode,
    .phy_get_duplex_mode = phy_lan8742_get_duplex_mode,
    .phy_get_partner_pause_enable = phy_mii_get_partner_pause_enable,
    .reset_timeout_ms = 1000,
    .promiscuous_enable = false,
};

// For debugging, return the values of all registers.
void phy_lan8742_read_registers(uint16_t values[32])
{
    int8_t i;
    for (i = 0; i <= 31; i++)
        values[i] = esp_eth_smi_read(i);
}

void phy_lan8742_dump_registers()
{
    ESP_LOGD(TAG, "LAN8742 Registers:");
    ESP_LOGD(TAG, "BCR    0x%04x", esp_eth_smi_read(0));
    ESP_LOGD(TAG, "BSR    0x%04x", esp_eth_smi_read(1));
    ESP_LOGD(TAG, "PHY1   0x%04x", esp_eth_smi_read(2));
    ESP_LOGD(TAG, "PHY2   0x%04x", esp_eth_smi_read(3));
    ESP_LOGD(TAG, "ANAR   0x%04x", esp_eth_smi_read(4));
    ESP_LOGD(TAG, "ANLPAR 0x%04x", esp_eth_smi_read(5));
    ESP_LOGD(TAG, "ANER   0x%04x", esp_eth_smi_read(6));
    ESP_LOGD(TAG, "EDPD   0x%04x", esp_eth_smi_read(16));
    ESP_LOGD(TAG, "MCSR   0x%04x", esp_eth_smi_read(17));
    ESP_LOGD(TAG, "SMR    0x%04x", esp_eth_smi_read(18));
    ESP_LOGD(TAG, "SECR   0x%04x", esp_eth_smi_read(26));
    ESP_LOGD(TAG, "CSIR   0x%04x", esp_eth_smi_read(27));
    ESP_LOGD(TAG, "CLR    0x%04x", esp_eth_smi_read(28));
    ESP_LOGD(TAG, "ISR    0x%04x", esp_eth_smi_read(29));
    ESP_LOGD(TAG, "IMR    0x%04x", esp_eth_smi_read(30));
    ESP_LOGD(TAG, "PSCSR  0x%04x", esp_eth_smi_read(31));
    ESP_LOGD(TAG, "WUCSR  0x%04x", phy_lan8742_read_mmd_register(3, 32784));
    ESP_LOGD(TAG, "ADDRA  0x%04x", phy_lan8742_read_mmd_register(3, 32865));
    ESP_LOGD(TAG, "ADDRB  0x%04x", phy_lan8742_read_mmd_register(3, 32866));
    ESP_LOGD(TAG, "ADDRC  0x%04x", phy_lan8742_read_mmd_register(3, 32867));
}

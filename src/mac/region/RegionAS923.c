/*!
 * \file      RegionAS923.c
 *
 * \brief     Region implementation for AS923
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
*/
#include "node/lora_utilities.h"
#include "RegionCommon.h"
#include "RegionAS923.h"

// Definitions
#define CHANNELS_MASK_SIZE              1

// AS923 Channel Plan. From https://github.com/Lora-net/LoRaMac-node/blob/master/src/mac/region/RegionAS923.c#L38-L97

#ifndef REGION_AS923_DEFAULT_CHANNEL_PLAN
#define REGION_AS923_DEFAULT_CHANNEL_PLAN CHANNEL_PLAN_GROUP_AS923_1
#endif

#if( REGION_AS923_DEFAULT_CHANNEL_PLAN == CHANNEL_PLAN_GROUP_AS923_1 )

// Channel plan CHANNEL_PLAN_GROUP_AS923_1

#define REGION_AS923_FREQ_OFFSET          0

#define AS923_MIN_RF_FREQUENCY            915000000
#define AS923_MAX_RF_FREQUENCY            928000000

#elif ( REGION_AS923_DEFAULT_CHANNEL_PLAN == CHANNEL_PLAN_GROUP_AS923_2 )

// Channel plan CHANNEL_PLAN_GROUP_AS923_2
// -1.8MHz
#define REGION_AS923_FREQ_OFFSET          ( ( ~( 0xFFFFB9B0 ) + 1 ) * 100 )

#define AS923_MIN_RF_FREQUENCY            915000000
#define AS923_MAX_RF_FREQUENCY            928000000

#elif ( REGION_AS923_DEFAULT_CHANNEL_PLAN == CHANNEL_PLAN_GROUP_AS923_3 )

// Channel plan CHANNEL_PLAN_GROUP_AS923_3
// -6.6MHz
#define REGION_AS923_FREQ_OFFSET          ( ( ~( 0xFFFEFE30 ) + 1 ) * 100 )

#define AS923_MIN_RF_FREQUENCY            915000000
#define AS923_MAX_RF_FREQUENCY            928000000

#elif ( REGION_AS923_DEFAULT_CHANNEL_PLAN == CHANNEL_PLAN_GROUP_AS923_1_JP )

// Channel plan CHANNEL_PLAN_GROUP_AS923_1_JP

#define REGION_AS923_FREQ_OFFSET          0

/*!
 * Restrict AS923 frequencies to channels 24 to 38
 * Center frequencies 920.6 MHz to 923.4 MHz @ 200 kHz max bandwidth
 */
#define AS923_MIN_RF_FREQUENCY            920600000
#define AS923_MAX_RF_FREQUENCY            923400000

/*!
 * Specifies the reception bandwidth to be used while executing the LBT
 * Max channel bandwidth is 200 kHz
 */
#define AS923_LBT_RX_BANDWIDTH            200000

#undef AS923_TX_MAX_DATARATE
#define AS923_TX_MAX_DATARATE             DR_5

#undef AS923_RX_MAX_DATARATE
#define AS923_RX_MAX_DATARATE             DR_5

#undef AS923_DEFAULT_MAX_EIRP
#define AS923_DEFAULT_MAX_EIRP            13.0f

#endif

// Global attributes
/*!
 * LoRaMAC channels
 */
static ChannelParams_t Channels[AS923_MAX_NB_CHANNELS];

/*!
 * LoRaMac bands
 */
static Band_t Bands[AS923_MAX_NB_BANDS] =
{
    AS923_BAND0
};

/*!
 * LoRaMac channels mask
 */
static uint16_t ChannelsMask[CHANNELS_MASK_SIZE];

/*!
 * LoRaMac channels default mask
 */
static uint16_t ChannelsDefaultMask[CHANNELS_MASK_SIZE];

// Static functions
static int8_t GetNextLowerTxDr( int8_t dr, int8_t minDr )
{
    uint8_t nextLowerDr = 0;

    if( dr == minDr )
    {
        nextLowerDr = minDr;
    }
    else
    {
        nextLowerDr = dr - 1;
    }
    return nextLowerDr;
}

static uint32_t GetBandwidth( uint32_t drIndex )
{
    switch( BandwidthsAS923[drIndex] )
    {
        default:
        case 125000:
            return 0;
        case 250000:
            return 1;
        case 500000:
            return 2;
    }
}

static int8_t LimitTxPower( int8_t txPower, int8_t maxBandTxPower, int8_t datarate, uint16_t* channelsMask )
{
    int8_t txPowerResult = txPower;

    // Limit tx power to the band max
    txPowerResult =  MAX( txPower, maxBandTxPower );

    return txPowerResult;
}

static bool VerifyTxFreq( uint32_t freq )
{
    // Check radio driver support
    if( Radio.CheckRfFrequency( freq ) == false )
    {
        return false;
    }

    if( ( freq < 915000000 ) || ( freq > 928000000 ) )
    {
        return false;
    }
    return true;
}

static uint8_t CountNbOfEnabledChannels( bool joined, uint8_t datarate, uint16_t* channelsMask, ChannelParams_t* channels, Band_t* bands, uint8_t* enabledChannels, uint8_t* delayTx )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTransmission = 0;

    for( uint8_t i = 0, k = 0; i < AS923_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( channelsMask[k] & ( 1 << j ) ) != 0 )
            {
                if( channels[i + j].Frequency == 0 )
                { // Check if the channel is enabled
                    continue;
                }
                if( joined == false )
                {
                    if( ( AS923_JOIN_CHANNELS & ( 1 << j ) ) == 0 )
                    {
                        continue;
                    }
                }
                if( RegionCommonValueInRange( datarate, channels[i + j].DrRange.Fields.Min,
                                              channels[i + j].DrRange.Fields.Max ) == false )
                { // Check if the current channel selection supports the given datarate
                    continue;
                }
                if( bands[channels[i + j].Band].TimeOff > 0 )
                { // Check if the band is available for transmission
                    delayTransmission++;
                    continue;
                }
                enabledChannels[nbEnabledChannels++] = i + j;
            }
        }
    }

    *delayTx = delayTransmission;
    return nbEnabledChannels;
}

PhyParam_t RegionAS923GetPhyParam( GetPhyParams_t* getPhy )
{
    PhyParam_t phyParam = { 0 };

    switch( getPhy->Attribute )
    {
        case PHY_MIN_RX_DR:
        {
            if( getPhy->DownlinkDwellTime == 0 )
            {
                phyParam.Value = AS923_RX_MIN_DATARATE;
            }
            else
            {
                phyParam.Value = AS923_DWELL_LIMIT_DATARATE;
            }
            break;
        }
        case PHY_MIN_TX_DR:
        {
            if( getPhy->UplinkDwellTime == 0 )
            {
                phyParam.Value = AS923_TX_MIN_DATARATE;
            }
            else
            {
                phyParam.Value = AS923_DWELL_LIMIT_DATARATE;
            }
            break;
        }
        case PHY_DEF_TX_DR:
        {
            phyParam.Value = AS923_DEFAULT_DATARATE;
            break;
        }
        case PHY_NEXT_LOWER_TX_DR:
        {
            if( getPhy->UplinkDwellTime == 0 )
            {
                phyParam.Value = GetNextLowerTxDr( getPhy->Datarate, AS923_TX_MIN_DATARATE );
            }
            else
            {
                phyParam.Value = GetNextLowerTxDr( getPhy->Datarate, AS923_DWELL_LIMIT_DATARATE );
            }
            break;
        }
        case PHY_DEF_TX_POWER:
        {
            phyParam.Value = AS923_DEFAULT_TX_POWER;
            break;
        }
        case PHY_MAX_PAYLOAD:
        {
            if( getPhy->UplinkDwellTime == 0 )
            {
                phyParam.Value = MaxPayloadOfDatarateDwell0AS923[getPhy->Datarate];
            }
            else
            {
                phyParam.Value = MaxPayloadOfDatarateDwell1UpAS923[getPhy->Datarate];
            }
            break;
        }
        case PHY_MAX_PAYLOAD_REPEATER:
        {
            if( getPhy->UplinkDwellTime == 0 )
            {
                phyParam.Value = MaxPayloadOfDatarateRepeaterDwell0AS923[getPhy->Datarate];
            }
            else
            {
                phyParam.Value = MaxPayloadOfDatarateDwell1UpAS923[getPhy->Datarate];
            }
            break;
        }
        case PHY_DUTY_CYCLE:
        {
            phyParam.Value = AS923_DUTY_CYCLE_ENABLED;
            break;
        }
        case PHY_MAX_RX_WINDOW:
        {
            phyParam.Value = AS923_MAX_RX_WINDOW;
            break;
        }
        case PHY_RECEIVE_DELAY1:
        {
            phyParam.Value = AS923_RECEIVE_DELAY1;
            break;
        }
        case PHY_RECEIVE_DELAY2:
        {
            phyParam.Value = AS923_RECEIVE_DELAY2;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY1:
        {
            phyParam.Value = AS923_JOIN_ACCEPT_DELAY1;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY2:
        {
            phyParam.Value = AS923_JOIN_ACCEPT_DELAY2;
            break;
        }
        case PHY_MAX_FCNT_GAP:
        {
            phyParam.Value = AS923_MAX_FCNT_GAP;
            break;
        }
        case PHY_ACK_TIMEOUT:
        {
            phyParam.Value = ( AS923_ACKTIMEOUT + randr( -AS923_ACK_TIMEOUT_RND, AS923_ACK_TIMEOUT_RND ) );
            break;
        }
        case PHY_DEF_DR1_OFFSET:
        {
            phyParam.Value = AS923_DEFAULT_RX1_DR_OFFSET;
            break;
        }
        case PHY_DEF_RX2_FREQUENCY:
        {
            phyParam.Value = AS923_RX_WND_2_FREQ;
            break;
        }
        case PHY_DEF_RX2_DR:
        {
            phyParam.Value = AS923_RX_WND_2_DR;
            break;
        }
        case PHY_CHANNELS_MASK:
        {
            phyParam.ChannelsMask = ChannelsMask;
            break;
        }
        case PHY_CHANNELS_DEFAULT_MASK:
        {
            phyParam.ChannelsMask = ChannelsDefaultMask;
            break;
        }
        case PHY_MAX_NB_CHANNELS:
        {
            phyParam.Value = AS923_MAX_NB_CHANNELS;
            break;
        }
        case PHY_CHANNELS:
        {
            phyParam.Channels = Channels;
            break;
        }
        case PHY_DEF_UPLINK_DWELL_TIME:
        {
            phyParam.Value = AS923_DEFAULT_UPLINK_DWELL_TIME;
            break;
        }
        case PHY_DEF_DOWNLINK_DWELL_TIME:
        {
            phyParam.Value = AS923_DEFAULT_DOWNLINK_DWELL_TIME;
            break;
        }
        case PHY_DEF_MAX_EIRP:
        {
            phyParam.fValue = AS923_DEFAULT_MAX_EIRP;
            break;
        }
        case PHY_DEF_ANTENNA_GAIN:
        {
            phyParam.fValue = AS923_DEFAULT_ANTENNA_GAIN;
            break;
        }
        default:
        {
            break;
        }
    }

    return phyParam;
}

void RegionAS923SetBandTxDone( SetBandTxDoneParams_t* txDone )
{
    RegionCommonSetBandTxDone( txDone->Joined, &Bands[Channels[txDone->Channel].Band], txDone->LastTxDoneTime );
}

void RegionAS923InitDefaults( InitType_t type )
{
    switch( type )
    {
        case INIT_TYPE_INIT:
        {
            // Channels
            Channels[0] = ( ChannelParams_t ) AS923_LC1;
            Channels[1] = ( ChannelParams_t ) AS923_LC2;

            // Initialize the channels default mask
            ChannelsDefaultMask[0] = LC( 1 ) + LC( 2 );
            // Update the channels mask
            RegionCommonChanMaskCopy( ChannelsMask, ChannelsDefaultMask, 1 );
            break;
        }
        case INIT_TYPE_RESTORE:
        {
            // Restore channels default mask
            ChannelsMask[0] |= ChannelsDefaultMask[0];
            break;
        }
        case INIT_TYPE_APP_DEFAULTS:
        {
            // Update the channels mask defaults
            RegionCommonChanMaskCopy( ChannelsMask, ChannelsDefaultMask, 1 );
            break;
        }
        default:
        {
            break;
        }
    }
}

bool RegionAS923Verify( VerifyParams_t* verify, PhyAttribute_t phyAttribute )
{
    switch( phyAttribute )
    {
        case PHY_TX_DR:
        {
            if( verify->DatarateParams.UplinkDwellTime == 0 )
            {
                return RegionCommonValueInRange( verify->DatarateParams.Datarate, AS923_TX_MIN_DATARATE, AS923_TX_MAX_DATARATE );
            }
            else
            {
                return RegionCommonValueInRange( verify->DatarateParams.Datarate, AS923_DWELL_LIMIT_DATARATE, AS923_TX_MAX_DATARATE );
            }
        }
        case PHY_DEF_TX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, DR_0, DR_5 );
        }
        case PHY_RX_DR:
        {
            if( verify->DatarateParams.DownlinkDwellTime == 0 )
            {
                return RegionCommonValueInRange( verify->DatarateParams.Datarate, AS923_RX_MIN_DATARATE, AS923_RX_MAX_DATARATE );
            }
            else
            {
                return RegionCommonValueInRange( verify->DatarateParams.Datarate, AS923_DWELL_LIMIT_DATARATE, AS923_RX_MAX_DATARATE );
            }
        }
        case PHY_DEF_TX_POWER:
        case PHY_TX_POWER:
        {
            // Remark: switched min and max!
            return RegionCommonValueInRange( verify->TxPower, AS923_MAX_TX_POWER, AS923_MIN_TX_POWER );
        }
        case PHY_DUTY_CYCLE:
        {
            return AS923_DUTY_CYCLE_ENABLED;
        }
        default:
            return false;
    }
}

void RegionAS923ApplyCFList( ApplyCFListParams_t* applyCFList )
{
    ChannelParams_t newChannel;
    ChannelAddParams_t channelAdd;
    ChannelRemoveParams_t channelRemove;

    // Setup default datarate range
    newChannel.DrRange.Value = ( DR_5 << 4 ) | DR_0;

    // Size of the optional CF list
    if( applyCFList->Size != 16 )
    {
        return;
    }

    // Last byte is RFU, don't take it into account
    for( uint8_t i = 0, chanIdx = AS923_NUMB_DEFAULT_CHANNELS; chanIdx < AS923_MAX_NB_CHANNELS; i+=3, chanIdx++ )
    {
        if( chanIdx < ( AS923_NUMB_CHANNELS_CF_LIST + AS923_NUMB_DEFAULT_CHANNELS ) )
        {
            // Channel frequency
            newChannel.Frequency = (uint32_t) applyCFList->Payload[i];
            newChannel.Frequency |= ( (uint32_t) applyCFList->Payload[i + 1] << 8 );
            newChannel.Frequency |= ( (uint32_t) applyCFList->Payload[i + 2] << 16 );
            newChannel.Frequency *= 100;

            // Initialize alternative frequency to 0
            newChannel.Rx1Frequency = 0;
        }
        else
        {
            newChannel.Frequency = 0;
            newChannel.DrRange.Value = 0;
            newChannel.Rx1Frequency = 0;
        }

        if( newChannel.Frequency != 0 )
        {
            channelAdd.NewChannel = &newChannel;
            channelAdd.ChannelId = chanIdx;

            // Try to add all channels
            RegionAS923ChannelAdd( &channelAdd );
        }
        else
        {
            channelRemove.ChannelId = chanIdx;

            RegionAS923ChannelsRemove( &channelRemove );
        }
    }
}

bool RegionAS923ChanMaskSet( ChanMaskSetParams_t* chanMaskSet )
{
    switch( chanMaskSet->ChannelsMaskType )
    {
        case CHANNELS_MASK:
        {
            RegionCommonChanMaskCopy( ChannelsMask, chanMaskSet->ChannelsMaskIn, 1 );
            break;
        }
        case CHANNELS_DEFAULT_MASK:
        {
            RegionCommonChanMaskCopy( ChannelsDefaultMask, chanMaskSet->ChannelsMaskIn, 1 );
            break;
        }
        default:
            return false;
    }
    return true;
}

bool RegionAS923AdrNext( AdrNextParams_t* adrNext, int8_t* drOut, int8_t* txPowOut, uint32_t* adrAckCounter )
{
    bool adrAckReq = false;
    int8_t datarate = adrNext->Datarate;
    int8_t minTxDatarate = 0;
    int8_t txPower = adrNext->TxPower;
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;

    // Get the minimum possible datarate
    getPhy.Attribute = PHY_MIN_TX_DR;
    getPhy.UplinkDwellTime = adrNext->UplinkDwellTime;
    phyParam = RegionAS923GetPhyParam( &getPhy );
    minTxDatarate = phyParam.Value;

    // Report back the adr ack counter
    *adrAckCounter = adrNext->AdrAckCounter;

    // Apply the minimum possible datarate.
    datarate = MAX( datarate, minTxDatarate );

    if( adrNext->AdrEnabled == true )
    {
        if( datarate == minTxDatarate )
        {
            *adrAckCounter = 0;
            adrAckReq = false;
        }
        else
        {
            if( adrNext->AdrAckCounter >= AS923_ADR_ACK_LIMIT )
            {
                adrAckReq = true;
                txPower = AS923_MAX_TX_POWER;
            }
            else
            {
                adrAckReq = false;
            }
            if( adrNext->AdrAckCounter >= ( AS923_ADR_ACK_LIMIT + AS923_ADR_ACK_DELAY ) )
            {
                if( ( adrNext->AdrAckCounter % AS923_ADR_ACK_DELAY ) == 1 )
                {
                    // Decrease the datarate
                    getPhy.Attribute = PHY_NEXT_LOWER_TX_DR;
                    getPhy.Datarate = datarate;
                    getPhy.UplinkDwellTime = adrNext->UplinkDwellTime;
                    phyParam = RegionAS923GetPhyParam( &getPhy );
                    datarate = phyParam.Value;

                    if( datarate == minTxDatarate )
                    {
                        // We must set adrAckReq to false as soon as we reach the lowest datarate
                        adrAckReq = false;
                        if( adrNext->UpdateChanMask == true )
                        {
                            // Re-enable default channels
                            ChannelsMask[0] |= LC( 1 ) + LC( 2 );
                        }
                    }
                }
            }
        }
    }

    *drOut = datarate;
    *txPowOut = txPower;
    return adrAckReq;
}

void RegionAS923ComputeRxWindowParameters( int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams )
{
    double tSymbol = 0.0;

    // Get the datarate, perform a boundary check
    rxConfigParams->Datarate = MIN( datarate, AS923_RX_MAX_DATARATE );
    rxConfigParams->Bandwidth = GetBandwidth( rxConfigParams->Datarate );

    if( rxConfigParams->Datarate == DR_7 )
    { // FSK
        tSymbol = RegionCommonComputeSymbolTimeFsk( DataratesAS923[rxConfigParams->Datarate] );
    }
    else
    { // LoRa
        tSymbol = RegionCommonComputeSymbolTimeLoRa( DataratesAS923[rxConfigParams->Datarate], BandwidthsAS923[rxConfigParams->Datarate] );
    }
    rxConfigParams->tsymbol = tSymbol;

    RegionCommonComputeRxWindowParameters( tSymbol, minRxSymbols, rxError, Radio.GetWakeupTime( ), &rxConfigParams->WindowTimeout, &rxConfigParams->WindowOffset );
}

bool RegionAS923RxConfig( RxConfigParams_t* rxConfig, int8_t* datarate )
{
    RadioModems_t modem;
    int8_t dr = rxConfig->Datarate;
    uint8_t maxPayload = 0;
    int8_t phyDr = 0;
    uint32_t frequency = rxConfig->Frequency;

    if( Radio.GetStatus( ) != RF_IDLE )
    {
        return false;
    }

    if( rxConfig->RxSlot == RX_SLOT_WIN_1 )
    {
        // Apply window 1 frequency
        frequency = Channels[rxConfig->Channel].Frequency;
        // Apply the alternative RX 1 window frequency, if it is available
        if( Channels[rxConfig->Channel].Rx1Frequency != 0 )
        {
            frequency = Channels[rxConfig->Channel].Rx1Frequency;
        }
    }

    // Read the physical datarate from the datarates table
    phyDr = DataratesAS923[dr];

    Radio.SetChannel( frequency );

    // Radio configuration
    if( dr == DR_7 )
    {
        modem = MODEM_FSK;
        Radio.SetRxConfig( modem, 50000, phyDr * 1000, 0, 83333, 5, rxConfig->WindowTimeout, false, 0, true, 0, 0, false, rxConfig->RxContinuous );
    }
    else
    {
        modem = MODEM_LORA;
        Radio.SetRxConfig( modem, rxConfig->Bandwidth, phyDr, 1, 0, 8, rxConfig->WindowTimeout, false, 0, false, 0, 0, true, rxConfig->RxContinuous );
    }

    // Check for repeater support
    if( rxConfig->RepeaterSupport == true )
    {
        maxPayload = MaxPayloadOfDatarateRepeaterDwell0AS923[dr];
    }
    else
    {
        maxPayload = MaxPayloadOfDatarateDwell0AS923[dr];
    }

    Radio.SetMaxPayloadLength( modem, maxPayload + LORA_MAC_FRMPAYLOAD_OVERHEAD );

    *datarate = (uint8_t) dr;
    return true;
}

bool RegionAS923TxConfig( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    printf("RegionAS923TxConfig\r\n");
    RadioModems_t modem;
    int8_t phyDr = DataratesAS923[txConfig->Datarate];
    int8_t txPowerLimited = LimitTxPower( txConfig->TxPower, Bands[Channels[txConfig->Channel].Band].TxMaxPower, txConfig->Datarate, ChannelsMask );
    uint32_t bandwidth = GetBandwidth( txConfig->Datarate );
    int8_t phyTxPower = 0;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, txConfig->MaxEirp, txConfig->AntennaGain );

    // Setup the radio frequency
    Radio.SetChannel( Channels[txConfig->Channel].Frequency );

    if( txConfig->Datarate == DR_7 )
    { // High Speed FSK channel
        modem = MODEM_FSK;
        Radio.SetTxConfig( modem, phyTxPower, 25000, bandwidth, phyDr * 1000, 0, 5, false, true, 0, 0, false, 3000 );
    }
    else
    {
        modem = MODEM_LORA;
        Radio.SetTxConfig( modem, phyTxPower, 0, bandwidth, phyDr, 1, 8, false, true, 0, 0, false, 3000 );
    }

    // Setup maximum payload lenght of the radio driver
    Radio.SetMaxPayloadLength( modem, txConfig->PktLen );
    // Get the time-on-air of the next tx frame
    //// TODO: Previously *txTimeOnAir = Radio.TimeOnAir( modem, txConfig->PktLen );
    *txTimeOnAir = Radio.TimeOnAir( MODEM_LORA, bandwidth, phyDr, 1, 8, false, txConfig->PktLen, true );

    *txPower = txPowerLimited;
    return true;
}

uint8_t RegionAS923LinkAdrReq( LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed )
{
    uint8_t status = 0x07;
    RegionCommonLinkAdrParams_t linkAdrParams;
    uint8_t nextIndex = 0;
    uint8_t bytesProcessed = 0;
    uint16_t chMask = 0;
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;
    RegionCommonLinkAdrReqVerifyParams_t linkAdrVerifyParams;

    while( bytesProcessed < linkAdrReq->PayloadSize )
    {
        // Get ADR request parameters
        nextIndex = RegionCommonParseLinkAdrReq( &( linkAdrReq->Payload[bytesProcessed] ), &linkAdrParams );

        if( nextIndex == 0 )
            break; // break loop, since no more request has been found

        // Update bytes processed
        bytesProcessed += nextIndex;

        // Revert status, as we only check the last ADR request for the channel mask KO
        status = 0x07;

        // Setup temporary channels mask
        chMask = linkAdrParams.ChMask;

        // Verify channels mask
        if( ( linkAdrParams.ChMaskCtrl == 0 ) && ( chMask == 0 ) )
        {
            status &= 0xFE; // Channel mask KO
        }
        else if( ( ( linkAdrParams.ChMaskCtrl >= 1 ) && ( linkAdrParams.ChMaskCtrl <= 5 )) ||
                ( linkAdrParams.ChMaskCtrl >= 7 ) )
        {
            // RFU
            status &= 0xFE; // Channel mask KO
        }
        else
        {
            for( uint8_t i = 0; i < AS923_MAX_NB_CHANNELS; i++ )
            {
                if( linkAdrParams.ChMaskCtrl == 6 )
                {
                    if( Channels[i].Frequency != 0 )
                    {
                        chMask |= 1 << i;
                    }
                }
                else
                {
                    if( ( ( chMask & ( 1 << i ) ) != 0 ) &&
                        ( Channels[i].Frequency == 0 ) )
                    {// Trying to enable an undefined channel
                        status &= 0xFE; // Channel mask KO
                    }
                }
            }
        }
    }

    // Get the minimum possible datarate
    getPhy.Attribute = PHY_MIN_TX_DR;
    getPhy.UplinkDwellTime = linkAdrReq->UplinkDwellTime;
    phyParam = RegionAS923GetPhyParam( &getPhy );

    linkAdrVerifyParams.Status = status;
    linkAdrVerifyParams.AdrEnabled = linkAdrReq->AdrEnabled;
    linkAdrVerifyParams.Datarate = linkAdrParams.Datarate;
    linkAdrVerifyParams.TxPower = linkAdrParams.TxPower;
    linkAdrVerifyParams.NbRep = linkAdrParams.NbRep;
    linkAdrVerifyParams.CurrentDatarate = linkAdrReq->CurrentDatarate;
    linkAdrVerifyParams.CurrentTxPower = linkAdrReq->CurrentTxPower;
    linkAdrVerifyParams.CurrentNbRep = linkAdrReq->CurrentNbRep;
    linkAdrVerifyParams.NbChannels = AS923_MAX_NB_CHANNELS;
    linkAdrVerifyParams.ChannelsMask = &chMask;
    linkAdrVerifyParams.MinDatarate = ( int8_t )phyParam.Value;
    linkAdrVerifyParams.MaxDatarate = AS923_TX_MAX_DATARATE;
    linkAdrVerifyParams.Channels = Channels;
    linkAdrVerifyParams.MinTxPower = AS923_MIN_TX_POWER;
    linkAdrVerifyParams.MaxTxPower = AS923_MAX_TX_POWER;

    // Verify the parameters and update, if necessary
    status = RegionCommonLinkAdrReqVerifyParams( &linkAdrVerifyParams, &linkAdrParams.Datarate, &linkAdrParams.TxPower, &linkAdrParams.NbRep );

    // Update channelsMask if everything is correct
    if( status == 0x07 )
    {
        // Set the channels mask to a default value
        memset1( ( uint8_t* )ChannelsMask, 0, sizeof( ChannelsMask ) );
        // Update the channels mask
        ChannelsMask[0] = chMask;
    }

    // Update status variables
    *drOut = linkAdrParams.Datarate;
    *txPowOut = linkAdrParams.TxPower;
    *nbRepOut = linkAdrParams.NbRep;
    *nbBytesParsed = bytesProcessed;

    return status;
}

uint8_t RegionAS923RxParamSetupReq( RxParamSetupReqParams_t* rxParamSetupReq )
{
    uint8_t status = 0x07;

    // Verify radio frequency
    if( Radio.CheckRfFrequency( rxParamSetupReq->Frequency ) == false )
    {
        status &= 0xFE; // Channel frequency KO
    }

    // Verify datarate
    if( RegionCommonValueInRange( rxParamSetupReq->Datarate, AS923_RX_MIN_DATARATE, AS923_RX_MAX_DATARATE ) == false )
    {
        status &= 0xFD; // Datarate KO
    }

    // Verify datarate offset
    if( RegionCommonValueInRange( rxParamSetupReq->DrOffset, AS923_MIN_RX1_DR_OFFSET, AS923_MAX_RX1_DR_OFFSET ) == false )
    {
        status &= 0xFB; // Rx1DrOffset range KO
    }

    return status;
}

uint8_t RegionAS923NewChannelReq( NewChannelReqParams_t* newChannelReq )
{
    uint8_t status = 0x03;
    ChannelAddParams_t channelAdd;
    ChannelRemoveParams_t channelRemove;

    if( newChannelReq->NewChannel->Frequency == 0 )
    {
        channelRemove.ChannelId = newChannelReq->ChannelId;

        // Remove
        if( RegionAS923ChannelsRemove( &channelRemove ) == false )
        {
            status &= 0xFC;
        }
    }
    else
    {
        channelAdd.NewChannel = newChannelReq->NewChannel;
        channelAdd.ChannelId = newChannelReq->ChannelId;

        switch( RegionAS923ChannelAdd( &channelAdd ) )
        {
            case LORAMAC_STATUS_OK:
            {
                break;
            }
            case LORAMAC_STATUS_FREQUENCY_INVALID:
            {
                status &= 0xFE;
                break;
            }
            case LORAMAC_STATUS_DATARATE_INVALID:
            {
                status &= 0xFD;
                break;
            }
            case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
            {
                status &= 0xFC;
                break;
            }
            default:
            {
                status &= 0xFC;
                break;
            }
        }
    }

    return status;
}

int8_t RegionAS923TxParamSetupReq( TxParamSetupReqParams_t* txParamSetupReq )
{
    // Accept the request
    return 0;
}

uint8_t RegionAS923DlChannelReq( DlChannelReqParams_t* dlChannelReq )
{
    uint8_t status = 0x03;

    // Verify if the frequency is supported
    if( VerifyTxFreq( dlChannelReq->Rx1Frequency ) == false )
    {
        status &= 0xFE;
    }

    // Verify if an uplink frequency exists
    if( Channels[dlChannelReq->ChannelId].Frequency == 0 )
    {
        status &= 0xFD;
    }

    // Apply Rx1 frequency, if the status is OK
    if( status == 0x03 )
    {
        Channels[dlChannelReq->ChannelId].Rx1Frequency = dlChannelReq->Rx1Frequency;
    }

    return status;
}

int8_t RegionAS923AlternateDr( int8_t currentDr )
{
    // Only AS923_DWELL_LIMIT_DATARATE is supported
    return AS923_DWELL_LIMIT_DATARATE;
}

void RegionAS923CalcBackOff( CalcBackOffParams_t* calcBackOff )
{
    RegionCommonCalcBackOffParams_t calcBackOffParams;

    calcBackOffParams.Channels = Channels;
    calcBackOffParams.Bands = Bands;
    calcBackOffParams.LastTxIsJoinRequest = calcBackOff->LastTxIsJoinRequest;
    calcBackOffParams.Joined = calcBackOff->Joined;
    calcBackOffParams.DutyCycleEnabled = calcBackOff->DutyCycleEnabled;
    calcBackOffParams.Channel = calcBackOff->Channel;
    calcBackOffParams.ElapsedTime = calcBackOff->ElapsedTime;
    calcBackOffParams.TxTimeOnAir = calcBackOff->TxTimeOnAir;

    RegionCommonCalcBackOff( &calcBackOffParams );
}

LoRaMacStatus_t RegionAS923NextChannel( NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff )
{
    printf("RegionAS923NextChannel\r\n");
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTx = 0;
    uint8_t enabledChannels[AS923_MAX_NB_CHANNELS] = { 0 };
    TimerTime_t nextTxDelay = 0;

    if( RegionCommonCountChannels( ChannelsMask, 0, 1 ) == 0 )
    { // Reactivate default channels
        ChannelsMask[0] |= LC( 1 ) + LC( 2 );
    }

    if( nextChanParams->AggrTimeOff <= TimerGetElapsedTime( nextChanParams->LastAggrTx ) )
    {
        // Reset Aggregated time off
        *aggregatedTimeOff = 0;

        // Update bands Time OFF
        nextTxDelay = RegionCommonUpdateBandTimeOff( nextChanParams->Joined, nextChanParams->DutyCycleEnabled, Bands, AS923_MAX_NB_BANDS );

        // Search how many channels are enabled
        nbEnabledChannels = CountNbOfEnabledChannels( nextChanParams->Joined, nextChanParams->Datarate,
                                                      ChannelsMask, Channels,
                                                      Bands, enabledChannels, &delayTx );
    }
    else
    {
        delayTx++;
        nextTxDelay = nextChanParams->AggrTimeOff - TimerGetElapsedTime( nextChanParams->LastAggrTx );
    }

    if( nbEnabledChannels > 0 )
    {
        //// Note: This the new Carrier Sensing implementation for AS923 Channel Plan,
        //// based on Semtech's Reference Implementation: https://github.com/Lora-net/LoRaMac-node/blob/master/src/mac/region/RegionAS923.c#L911-L935
        //// Only Japan and South Korea need Carrier Sensing to select a LoRa
        //// Frequency Channel before transmitting. Other regions can select
        //// a random LoRa Frequency Channel without Carrier Sensing.
#if ( REGION_AS923_DEFAULT_CHANNEL_PLAN == CHANNEL_PLAN_GROUP_AS923_1_JP )
        // Carrier Sensing in Japan: Executes the LBT algorithm when operating in Japan
        uint8_t channelNext = 0;

        for( uint8_t  i = 0, j = randr( 0, nbEnabledChannels - 1 ); i < AS923_MAX_NB_CHANNELS; i++ )
        {
            channelNext = enabledChannels[j];
            j = ( j + 1 ) % nbEnabledChannels;

            //// TODO: Carrier Sensing needs to fixed because Radio.IsChannelFree has different parameters for SX1262 and SX1276
            #error Radio.IsChannelFree interface is different for SX1262 and SX1276
            printf("Radio.IsChannelFree interface is different for SX1262 and SX1276\r\n");
            assert(false);

            // Perform carrier sense for AS923_CARRIER_SENSE_TIME
            // If the channel is free, we can stop the LBT mechanism
            if( Radio.IsChannelFree( RegionNvmGroup2->Channels[channelNext].Frequency, AS923_LBT_RX_BANDWIDTH, AS923_RSSI_FREE_TH, AS923_CARRIER_SENSE_TIME ) == true )
            {
                // Free channel found
                *channel = channelNext;
                return LORAMAC_STATUS_OK;
            }
        }
        // Even if one or more channels are available according to the channel plan, no free channel
        // was found during the LBT procedure.
        printf("RegionAS923NextChannel: no free channel\r\n");
        return LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND;
#else
        // Outside Japan: Select a random LoRa Frequency Channel without Carrier Sensing
        *channel = enabledChannels[randr( 0, nbEnabledChannels - 1 )];
        printf("RegionAS923NextChannel: channel=%d\r\n", (int) *channel);
        return LORAMAC_STATUS_OK;
#endif

#ifdef NOTUSED
        //// Note: This is the old code from Mynewt without AS923 Channel Plan.
        //// The previous code from Mynewt will always perform Carrier Sensing.
        uint8_t channelNext = 0;
        for( uint8_t  i = 0, j = randr( 0, nbEnabledChannels - 1 ); i < AS923_MAX_NB_CHANNELS; i++ )
        {
            channelNext = enabledChannels[j];
            j = ( j + 1 ) % nbEnabledChannels;

            //// TODO: Carrier Sensing needs to fixed because Radio.IsChannelFree has different parameters for SX1262 and SX1276
            #error Radio.IsChannelFree interface is different for SX1262 and SX1276
            printf("Radio.IsChannelFree interface is different for SX1262 and SX1276\r\n");
            assert(false);

            // Perform carrier sense for AS923_CARRIER_SENSE_TIME
            // If the channel is free, we can stop the LBT mechanism
            if( Radio.IsChannelFree( MODEM_LORA, Channels[channelNext].Frequency, AS923_RSSI_FREE_TH, AS923_CARRIER_SENSE_TIME ) == true )
            {
                // Free channel found
                *channel = channelNext;
                *time = 0;
                printf("RegionAS923NextChannel: channel=%d\r\n", (int) *channel);
                return LORAMAC_STATUS_OK;
            }
        }
        printf("RegionAS923NextChannel: no free channel\r\n");
        return LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND;
#endif  //  NOTUSED

    }
    else
    {
        if( delayTx > 0 )
        {
            // Delay transmission due to AggregatedTimeOff or to a band time off
            *time = nextTxDelay;
            return LORAMAC_STATUS_DUTYCYCLE_RESTRICTED;
        }
        // Datarate not supported by any channel, restore defaults
        ChannelsMask[0] |= LC( 1 ) + LC( 2 );
        *time = 0;
        printf("RegionAS923NextChannel: no channel\r\n");
        return LORAMAC_STATUS_NO_CHANNEL_FOUND;
    }
}

LoRaMacStatus_t RegionAS923ChannelAdd( ChannelAddParams_t* channelAdd )
{
    uint8_t band = 0;
    bool drInvalid = false;
    bool freqInvalid = false;
    uint8_t id = channelAdd->ChannelId;

    if( id >= AS923_MAX_NB_CHANNELS )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    // Validate the datarate range
    if( RegionCommonValueInRange( channelAdd->NewChannel->DrRange.Fields.Min, AS923_TX_MIN_DATARATE, AS923_TX_MAX_DATARATE ) == false )
    {
        drInvalid = true;
    }
    if( RegionCommonValueInRange( channelAdd->NewChannel->DrRange.Fields.Max, AS923_TX_MIN_DATARATE, AS923_TX_MAX_DATARATE ) == false )
    {
        drInvalid = true;
    }
    if( channelAdd->NewChannel->DrRange.Fields.Min > channelAdd->NewChannel->DrRange.Fields.Max )
    {
        drInvalid = true;
    }

    // Default channels don't accept all values
    if( id < AS923_NUMB_DEFAULT_CHANNELS )
    {
        // Validate the datarate range for min: must be DR_0
        if( channelAdd->NewChannel->DrRange.Fields.Min > DR_0 )
        {
            drInvalid = true;
        }
        // Validate the datarate range for max: must be DR_5 <= Max <= TX_MAX_DATARATE
        if( RegionCommonValueInRange( channelAdd->NewChannel->DrRange.Fields.Max, DR_5, AS923_TX_MAX_DATARATE ) == false )
        {
            drInvalid = true;
        }
        // We are not allowed to change the frequency
        if( channelAdd->NewChannel->Frequency != Channels[id].Frequency )
        {
            freqInvalid = true;
        }
    }

    // Check frequency
    if( freqInvalid == false )
    {
        if( VerifyTxFreq( channelAdd->NewChannel->Frequency ) == false )
        {
            freqInvalid = true;
        }
    }

    // Check status
    if( ( drInvalid == true ) && ( freqInvalid == true ) )
    {
        return LORAMAC_STATUS_FREQ_AND_DR_INVALID;
    }
    if( drInvalid == true )
    {
        return LORAMAC_STATUS_DATARATE_INVALID;
    }
    if( freqInvalid == true )
    {
        return LORAMAC_STATUS_FREQUENCY_INVALID;
    }

    memcpy1( ( uint8_t* )( Channels + id ), ( uint8_t* )channelAdd->NewChannel, sizeof( Channels[id] ) );
    Channels[id].Band = band;
    ChannelsMask[0] |= ( 1 << id );
    return LORAMAC_STATUS_OK;
}

bool RegionAS923ChannelsRemove( ChannelRemoveParams_t* channelRemove  )
{
    uint8_t id = channelRemove->ChannelId;

    if( id < AS923_NUMB_DEFAULT_CHANNELS )
    {
        return false;
    }

    // Remove the channel from the list of channels
    Channels[id] = ( ChannelParams_t ){ 0, 0, { 0 }, 0 };

    return RegionCommonChanDisable( ChannelsMask, id, AS923_MAX_NB_CHANNELS );
}

void RegionAS923SetContinuousWave( ContinuousWaveParams_t* continuousWave )
{
    int8_t txPowerLimited = LimitTxPower( continuousWave->TxPower, Bands[Channels[continuousWave->Channel].Band].TxMaxPower, continuousWave->Datarate, ChannelsMask );
    int8_t phyTxPower = 0;
    uint32_t frequency = Channels[continuousWave->Channel].Frequency;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, continuousWave->MaxEirp, continuousWave->AntennaGain );

    Radio.SetTxContinuousWave( frequency, phyTxPower, continuousWave->Timeout );
}

uint8_t RegionAS923ApplyDrOffset( uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset )
{
    // Initialize minDr for a downlink dwell time configuration of 0
    int8_t minDr = DR_0;

    // Update the minDR for a downlink dwell time configuration of 1
    if( downlinkDwellTime == 1 )
    {
        minDr = AS923_DWELL_LIMIT_DATARATE;
    }

    // Apply offset formula
    return MIN( DR_5, MAX( minDr, dr - EffectiveRx1DrOffsetAS923[drOffset] ) );
}

/**
 * @file	ADConverter.cpp
 * @author	Kevin WYSOCKI
 * @date	23 mai 2017
 * @brief	A/D Converter class
 */

#include "ADConverter.hpp"



using namespace Utils;
using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

// PC3 - ADC123_IN13
#define ADC_CH0_INPUT_PORT      (GPIOC)
#define ADC_CH0_INPUT_PIN       (GPIO_Pin_3)
#define ADC_CH0_ADC             (ADC3)
#define ADC_CH0_ADC_CHANNEL     (ADC_Channel_13)

// PC4 - ADC123_IN14
#define ADC_CH1_INPUT_PORT      (GPIOC)
#define ADC_CH1_INPUT_PIN       (GPIO_Pin_4)
#define ADC_CH1_ADC             (ADC2)
#define ADC_CH1_ADC_CHANNEL     (ADC_Channel_14)

// PC5 - ADC123_IN15
#define ADC_CH2_INPUT_PORT      (GPIOC)
#define ADC_CH2_INPUT_PIN       (GPIO_Pin_5)
#define ADC_CH2_ADC             (ADC1)
#define ADC_CH2_ADC_CHANNEL     (ADC_Channel_15)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static ADConverter* _instance[ADConverter::Channel::ADC_ChannelMAX] = {NULL};

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @brief Retrieve Encoder definitions from Encoder ID
 * @param id : Encoder ID
 * @return ENC_DEF structure
 */
static ADC_DEF _getADCStruct (enum ADConverter::Channel channel)
{
    ADC_DEF adc;

    assert(channel < ADConverter::ADC_ChannelMAX);

    switch(channel)
    {
    case HAL::ADConverter::ADC_Channel0:
        adc.Input.PORT                  =   ADC_CH0_INPUT_PORT;
        adc.Input.PIN                   =   ADC_CH0_INPUT_PIN;
        adc.ADConverter.ADConverter     =   ADC_CH0_ADC;
        adc.ADConverter.Channel         =   ADC_CH0_ADC_CHANNEL;
        break;
    case HAL::ADConverter::ADC_Channel1:
        adc.Input.PORT                  =   ADC_CH1_INPUT_PORT;
        adc.Input.PIN                   =   ADC_CH1_INPUT_PIN;
        adc.ADConverter.ADConverter     =   ADC_CH1_ADC;
        adc.ADConverter.Channel         =   ADC_CH1_ADC_CHANNEL;
        break;
    case HAL::ADConverter::ADC_Channel2:
        adc.Input.PORT                  =   ADC_CH2_INPUT_PORT;
        adc.Input.PIN                   =   ADC_CH2_INPUT_PIN;
        adc.ADConverter.ADConverter     =   ADC_CH2_ADC;
        adc.ADConverter.Channel         =   ADC_CH2_ADC_CHANNEL;
        break;

    default:
        break;
    }

    return adc;
}

/**
 * @brief Initialize peripheral for a specific Encoder
 * @param id : Encoder ID
 */
static void _hardwareInit (enum ADConverter::Channel channel)
{
    GPIO_InitTypeDef GPIOStruct;
    ADC_InitTypeDef ADCStruct;
    ADC_CommonInitTypeDef ADCCommonStruct;

    ADC_DEF def;

    assert(channel < ADConverter::ADC_ChannelMAX);

    def = _getADCStruct(channel);

    // Init input
    GPIOStruct.GPIO_Mode    =   GPIO_Mode_AN;
    GPIOStruct.GPIO_OType   =   GPIO_OType_PP;
    GPIOStruct.GPIO_PuPd    =   GPIO_PuPd_NOPULL;
    GPIOStruct.GPIO_Speed   =   GPIO_High_Speed;
    GPIOStruct.GPIO_Pin     =   def.Input.PIN;

    GPIO_Init(def.Input.PORT, &GPIOStruct);

    // ADC Init
    ADCCommonStruct.ADC_Mode                =   ADC_Mode_Independent;
    ADCCommonStruct.ADC_Prescaler           =   ADC_Prescaler_Div4;
    ADCCommonStruct.ADC_DMAAccessMode       =   ADC_DMAAccessMode_Disabled;
    ADCCommonStruct.ADC_TwoSamplingDelay    =   0u;

    ADC_Cmd(def.ADConverter.ADConverter, DISABLE);

    ADC_CommonInit(&ADCCommonStruct);

    ADCStruct.ADC_ContinuousConvMode        =   DISABLE;
    ADCStruct.ADC_Resolution                =   ADC_Resolution_12b;
    ADCStruct.ADC_ExternalTrigConv          =   ADC_ExternalTrigConv_T1_CC1;    // Not used
    ADCStruct.ADC_ExternalTrigConvEdge      =   ADC_ExternalTrigConvEdge_None;
    ADCStruct.ADC_DataAlign                 =   ADC_DataAlign_Right;
    ADCStruct.ADC_ScanConvMode              =   DISABLE;
    ADCStruct.ADC_NbrOfConversion           =   1u;

    ADC_Init(def.ADConverter.ADConverter, &ADCStruct);

    ADC_RegularChannelConfig(def.ADConverter.ADConverter,
                             def.ADConverter.Channel,
                             1u,
                             ADC_SampleTime_15Cycles);

    ADC_Cmd(def.ADConverter.ADConverter, ENABLE);
}

/*----------------------------------------------------------------------------*/
/* Class Implementation	                                                      */
/*----------------------------------------------------------------------------*/

ADConverter* ADConverter::GetInstance(ADConverter::Channel channel)
{
    assert(channel < ADConverter::Channel::ADC_ChannelMAX);

    if(_instance[channel] == NULL)
    {
        _instance[channel] = new ADConverter(channel);
    }

    return _instance[channel];
}

ADConverter::ADConverter(ADConverter::Channel channel)
{
    this->result    =   0u;
    this->def       =   _getADCStruct(channel);

    _hardwareInit(channel);
}

int32_t ADConverter::StartConv()
{
    int32_t error = NO_ERROR;

    if(ADC_GetSoftwareStartConvStatus(this->def.ADConverter.ADConverter) == SET)
    {
        error = ADC_ERROR_BUSY;
    }
    else
    {
        ADC_SoftwareStartConv(this->def.ADConverter.ADConverter);
    }

    return error;
}

void ADConverter::WaitWhileBusy()
{
    while(ADC_GetSoftwareStartConvStatus(this->def.ADConverter.ADConverter) == SET);
}

uint16_t ADConverter::GetResult()
{
    this->result = ADC_GetConversionValue(this->def.ADConverter.ADConverter);

    return this->result;
}



/**
 * @file     pdb.h (180.ARM_Peripherals/Project_Headers/pdb.h)
 * @brief    Programmable Delay Block
 *
 * @version  V4.12.1.240
 * @date     28/10/2018
 */

#ifndef HEADER_PDB_H
#define HEADER_PDB_H
 /*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "hardware.h"

namespace USBDM {

/**
 * @addtogroup PDB_Group PDB, Programmable Delay Block
 * @brief Peripheral information for Programmable Delay Block
 * @{
 */
/**
 * Type definition for PDB interrupt call back
 */
typedef void (*PDBCallbackFunction)();

/**
 * Select the PDB clock pre-scale which affects counter speed
 */
enum PdbPrescale {
   PdbPrescale_1      = PDB_SC_PRESCALER(0),   //!< Divide by 1
   PdbPrescale_2      = PDB_SC_PRESCALER(1),   //!< Divide by 2
   PdbPrescale_4      = PDB_SC_PRESCALER(2),   //!< Divide by 4
   PdbPrescale_8      = PDB_SC_PRESCALER(3),   //!< Divide by 8
   PdbPrescale_16     = PDB_SC_PRESCALER(4),   //!< Divide by 16
   PdbPrescale_32     = PDB_SC_PRESCALER(5),   //!< Divide by 32
   PdbPrescale_64     = PDB_SC_PRESCALER(6),   //!< Divide by 64
   PdbPrescale_128    = PDB_SC_PRESCALER(7),   //!< Divide by 128
};

/**
 * Select the PDB clock pre-scale multiplier
 */
enum PdbMultiplier {
   PdbMultiplier_1      = PDB_SC_MULT(0),   //!< Prescaler multiplied by 1
   PdbMultiplier_10     = PDB_SC_MULT(1),   //!< Prescaler multiplied by 10
   PdbMultiplier_20     = PDB_SC_MULT(2),   //!< Prescaler multiplied by 20
   PdbMultiplier_40     = PDB_SC_MULT(3),   //!< Prescaler multiplied by 40
};

/**
 * Selects the PDB Trigger source
 */
enum PdbTrigger {
   PdbTrigger_External     = PDB_SC_TRGSEL(0),   //!< External Trigger Source PDBx_EXTRG
   PdbTrigger_Cmp0         = PDB_SC_TRGSEL(1),   //!< Comparator 0
   PdbTrigger_Cmp1         = PDB_SC_TRGSEL(2),   //!< Comparator 1
   PdbTrigger_3            = PDB_SC_TRGSEL(3),   //!< Reserved
   PdbTrigger_PitCh0       = PDB_SC_TRGSEL(4),   //!< PIT Channel 0
   PdbTrigger_PitCh1       = PDB_SC_TRGSEL(5),   //!< PIT Channel 1
   PdbTrigger_PitCh2       = PDB_SC_TRGSEL(6),   //!< PIT Channel 2
   PdbTrigger_PitCh3       = PDB_SC_TRGSEL(7),   //!< PIT Channel 3
   PdbTrigger_Ftm0         = PDB_SC_TRGSEL(8),   //!< FTM0 Init and Ext Trigger Outputs
   PdbTrigger_Ftm1         = PDB_SC_TRGSEL(9),   //!< FTM1 Init and Ext Trigger Outputs
   PdbTrigger_10           = PDB_SC_TRGSEL(10),  //!< Reserved
   PdbTrigger_11           = PDB_SC_TRGSEL(11),  //!< Reserved
   PdbTrigger_RtcAlarm     = PDB_SC_TRGSEL(12),  //!< RTC Alarm
   PdbTrigger_RtcSeconds   = PDB_SC_TRGSEL(13),  //!< RTC Seconds
   PdbTrigger_Lptrm        = PDB_SC_TRGSEL(14),  //!< LPTMR
   PdbTrigger_Software     = PDB_SC_TRGSEL(15),  //!< Software Trigger
};

/**
 * Controls the loading of MOD, IDLY, CHnDLYm, DACINTx,and POyDLY register from holding registers
 */
enum PdbLoadMode {
   PdbLoadMode_Immediate = PDB_SC_LDMOD(0), //!< Loaded immediately after LD_OK is set
   PdbLoadMode_Modulo    = PDB_SC_LDMOD(1), //!< Loaded on counter roll-over after LD_OK is set
   PdbLoadMode_Event     = PDB_SC_LDMOD(2), //!< Loaded on trigger event after LD_OK is set
   PdbLoadMode_Both      = PDB_SC_LDMOD(3), //!< Loaded on counter roll-over or trigger event after LD_OK is set
};

/**
 * Controls whether the PDB operates in one-shot or continuous mode
 */
enum PdbMode {
   PdbMode_OneShot    = PDB_SC_CONT(0),  //!< Sequence runs once only
   PdbMode_Continuous = PDB_SC_CONT(1),  //!< Sequence runs continuously once triggered
};

/**
 * Action done on event (PDBIF set)
 */
enum PdbAction {
   PdbAction_None       = PDB_SC_DMAEN(0)|PDB_SC_PDBIE(0), //!< No action on PDBIF set
   PdbAction_Interrupt  = PDB_SC_DMAEN(0)|PDB_SC_PDBIE(1), //!< Interrupt on PDBIF set
   PdbAction_Dma        = PDB_SC_DMAEN(1)|PDB_SC_PDBIE(1), //!< DMA on PDBIF set
};

/**
 * Controls whether the PDB error interrupt is enabled
 */
enum PdbErrorInterrupt {
   PdbErrorInterrupt_Disabled = PDB_SC_PDBEIE(0),   //!< Sequence error disabled
   PdbErrorInterrupt_Enabled  = PDB_SC_PDBEIE(1),   //!< Sequence error enabled
};

/**
 * Pretrigger control
 */
enum PdbPretrigger {
   PdbPretrigger_Disabled    = PDB_C1_EN(0),                //!< Pretrigger disabled
   PdbPretrigger_Bypassed    = PDB_C1_EN(1)|PDB_C1_TOS(0),  //!< Pretrigger asserts 1 clock after trigger
   PdbPretrigger_Delayed     = PDB_C1_EN(1)|PDB_C1_TOS(1),  //!< Pretrigger asserts 1 clock + delay after trigger
   PdbPretrigger_BackToBack  = PDB_C1_EN(1)|PDB_C1_BB(1),   //!< Back-to-back, pretrigger asserts 2 clocks after previous acknowledge
};

#if PDB_DAC_COUNT>0
/**
 * DAC Trigger Control
 *
 * _Disabled
 *    No DAC trigger is generated
 *
 * _Delayed
 *    DAC interval counter is reset and counting starts when a rising edge is detected on
 *    selected trigger input source or software trigger is selected and SWTRIG is written with 1.
 *
 * _External
 *    DAC interval counter is bypassed and DAC external trigger input triggers the DAC interval trigger.
 */
enum PdbDacTriggerMode {
   PdbDacTriggerMode_Disabled = PDB_INTC_TOE(0)|PDB_INTC_EXT(0), //!< No DAC trigger
   PdbDacTriggerMode_Delayed  = PDB_INTC_TOE(1)|PDB_INTC_EXT(0), //!< DAC trigger delayed by DAC interval counter
   PdbDacTriggerMode_External = PDB_INTC_TOE(1)|PDB_INTC_EXT(1), //!< DAC trigger is connected directly to external trigger
};
#endif

/**
 * Notes on the PDB.
 *
 * Components:\n
 *     - Channels+pre-triggers - Usually associated with an ADC
 *     - DAC delays            - Associated with a DAC
 *     - Pulse outputs         - Usually associated with a CMP
 *
 * Operation:\n
 * - The PDB can be triggered from another peripheral or directly by software using softwareTrigger().
 * - When triggered, the PDB counter and the DAC delay counters are restarted.
 * - The period of the PDB counter and the delay counters may be independently controlled.
 *    - The PDB counter counts [0..MOD]. Set with setPeriod().
 *    - The DAC delay counters count [0..DACINTx]. Set with configureDacTrigger().
 *      The counters may be bypassed for external triggers.
 * - The PDB may operate in one-shot or continuous mode.
 *    - One shot mode   - The PDB and DAC delay counters are reset on trigger. The PDB counter counts one sequence only.
 *      The DAC counter only operates while the main counter is counting but it may produce multiple triggers in the sequence.
 *    - Continuous mode - As above but the counter resets when it reaches the counter period and restarts.\n
 *      The DAC delay counters are NOT reset on PDB counter roll-over so operate independently after initial trigger.
 * - I would expect the DAC Delay counter period to be less than or equal to the PDB counter period in one-shot mode.
 * - The DAC Delay period may be set smaller than (PDB counter period/2) to trigger multiple DAC output
 *   triggers within a PDB period but more often there would be only a single DAC event e.g.
 *   <b>(PDB counter period/2) < (DAC Delay period) <= (PDB counter period)</b>. Again this is in one-shot mode.
 * - The pre-triggers within a channel are associated with different ADC pre-triggers e.g. SC1[n]/R[n]. The pre-triggers
 *   are referenced to the main PDB counter.\n
 *   This allows multiple ADC channels (inputs) to be used with a PDB period - usually only 2 are available per ADC.
 *   Set with configureAdcPretrigger().
 * - The pulse outputs may be configured as high for a pulse from [start...end] times based on the PDB counter.
 *   Set with configurePulseOutput().
 */
/**
 * Template class providing interface to Programmable Delay Block
 *
 * @tparam info      Information class for PDB
 *
 * @code
 * using pdb = PdbBase_T<PdbInfo>;
 *
 *  pdb::configure();
 *
 * @endcode
 */
template <class Info>
class PdbBase_T {

protected:
   /** Callback function for ISR */
   static PDBCallbackFunction sCallback;

   /** Callback function for error ISR */
   static PDBCallbackFunction sErrorCallback;

   /** Handler for unexpected interrupts */
   static void unhandledCallback() {
      setAndCheckErrorCode(E_NO_HANDLER);
   }
public:
   /**
    * IRQ handler
    */
   static void irqHandler(void) {

      if (PdbBase_T<Info>::pdb().SC & PDB_SC_PDBIF_MASK) {
         // Clear interrupt flag
         PdbBase_T<Info>::pdb().SC  &= ~PDB_SC_PDBIF_MASK;
         // Handle expected interrupt
         sCallback();
         return;
      }
      // Assume sequence error
      sErrorCallback();
   }

   /**
    * Set Callback function
    *
    *   @param[in]  callback Callback function to be executed on interrupt\n
    *                        Use nullptr to remove callback.
    */
   static void setCallback(PDBCallbackFunction callback) {

      static_assert(Info::irqHandlerInstalled, "PDB not configure for interrupts");
      if (callback == nullptr) {
         callback = unhandledCallback;
      }
      sCallback = callback;
   }

   /**
    * Set Callback function
    *
    *   @param[in]  callback Callback function to be executed on error interrupt\n
    *                        Use nullptr to remove callback.
    */
   static void setErrorCallback(PDBCallbackFunction callback) {

      static_assert(Info::irqHandlerInstalled, "PDB not configure for interrupts");
      if (callback == nullptr) {
         callback = unhandledCallback;
      }
      sErrorCallback = callback;
   }


protected:
   /** Hardware instance pointer */
   static volatile PDB_Type &pdb() { return Info::pdb(); }

public:
   /**
    * Configures all mapped pins associated with this peripheral
    */
   static void configureAllPins() {
      // Configure pins
      Info::initPCRs();
   }

   /**
    * Basic enable of PDB.
    * Includes enabling clock and configuring all pins of mapPinsOnEnable is selected on configuration
    */
   static void  enable() {

      if (Info::mapPinsOnEnable) {
         configureAllPins();
      }
      Info::enableClock();
      __DMB();
   }

   /**
    * Disable PDB
    */
   static void  disable() {

      pdb().SC  = 0;
      Info::disableClock();
      __DMB();
   }

   /**
    * Enables PDB and sets to default configuration.
    *
    * Includes enabling clock and any pins used.\n
    * Sets PDB to default configuration.
    */
   static void defaultConfigure() {

      enable();

      pdb().MOD  = Info::pdb_mod;
      pdb().IDLY = Info::pdb_idly;
      pdb().CH[0].C1     = Info::pdb_ch[0].c1;
      pdb().CH[0].DLY[0] = Info::pdb_ch[0].dly0;
      pdb().CH[0].DLY[1] = Info::pdb_ch[0].dly1;
      if (Info::numChannels>1) {
         pdb().CH[1].C1     = Info::pdb_ch[1].c1;
         pdb().CH[1].DLY[0] = Info::pdb_ch[1].dly0;
         pdb().CH[1].DLY[1] = Info::pdb_ch[1].dly1;
      }
#if PDB_DAC_COUNT>0
      if (Info::numDacs>0) {
         pdb().DAC[0].INTC = Info::pdb_dac[0].dacintc;
         pdb().DAC[0].INT  = Info::pdb_dac[0].dacint;
      }
      if (Info::numDacs>1) {
         pdb().DAC[1].INTC = Info::pdb_dac[1].dacintc;
         pdb().DAC[1].INT  = Info::pdb_dac[1].dacint;
      }
#endif
#ifdef PDB_POEN_POEN
      pdb().POEN = Info::pdb_poen;
      if (Info::numPulseOutputs>0) {
         pdb().POnDLY[0].PODLY     = Info::pdb_podly[0];
      }
      if (Info::numPulseOutputs>1) {
         pdb().POnDLY[1].PODLY     = Info::pdb_podly[1];
      }
      if (Info::numPulseOutputs>2) {
         pdb().POnDLY[2].PODLY     = Info::pdb_podly[2];
      }
      if (Info::numPulseOutputs>3) {
         pdb().POnDLY[3].PODLY     = Info::pdb_podly[3];
      }
#endif
      // Configure and trigger register load
      pdb().SC = Info::pdb_sc|PDB_SC_PDBEN_MASK|PDB_SC_LDOK_MASK;

      enableNvicInterrupts(Info::irqLevel);
   }

   /**
    * Configures the PDB
    *
    * Includes enabling clock and configuring all pins if mapPinsOnEnable is
    * selected in configuration.\n
    * ADC triggers, CMP pulse outputs and ADC triggers are disabled.
    *
    * @param pdbMode       PDB operates in one-shot or continuous mode
    * @param pdbTrigger    PDB Trigger source
    * @param pdbAction     Action done on event (PDBIF set)
    *
    * @note The PDB is NOT enabled.  This is usually done by confirmRegisterLoad().
    */
   static void configure(
         PdbMode     pdbMode,
         PdbTrigger  pdbTrigger,
         PdbAction   pdbAction = PdbAction_None
   ) {

      enable();
      pdb().SC = pdbMode|pdbTrigger|pdbAction;

      for (unsigned index=0; index<(sizeof(pdb().CH)/sizeof(pdb().CH[0])); index++) {
         pdb().CH[index].C1 = 0;
      }

#if PDB_DAC_COUNT>0
      for (unsigned index=0; index<(sizeof(pdb().DAC)/sizeof(pdb().DAC[0])); index++) {
         pdb().DAC[index].INTC = 0;
      }
#endif

#if PDB_POnDLY_COUNT>0
      pdb().POEN = 0;
#endif
   }

   /**
    * Set Interrupts and DMA actions
    *
    * @param[in] pdbAction          Controls action done on event (counter value is equal to the IDLY register)
    * @param[in] pdbErrorInterrupt  Controls sequence error interrupt requests (on any ADC sequence errors)
    */
   static void setActions(
         PdbAction            pdbAction,
         PdbErrorInterrupt    pdbErrorInterrupt = PdbErrorInterrupt_Disabled) {

      pdb().SC =
            (pdb().SC&~(PDB_SC_PDBIE_MASK|PDB_SC_PDBEIE_MASK|PDB_SC_DMAEN_MASK))|
            pdbAction|pdbErrorInterrupt|PDB_SC_PDBIF_MASK;
   }

   /**
    * Enable sequence error interrupts (pdb_sc_pdbeie)
    */
   static void enableErrorInterrupts() {
      pdb().SC |= PDB_SC_PDBEIE_MASK;
   }

   /**
    * Disable sequence error interrupts (pdb_sc_pdbeie)
    */
   static void disableErrorInterrupts() {

      pdb().SC &= ~PDB_SC_PDBEIE_MASK;
   }

   /**
    * Enable interrupts in NVIC
    */
   static void enableNvicInterrupts() {
      NVIC_EnableIRQ(Info::irqNums[0]);
   }

   /**
    * Enable and set priority of interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    *
    * @param[in]  nvicPriority  Interrupt priority
    */
   static void enableNvicInterrupts(uint32_t nvicPriority) {
      enableNvicInterrupt(Info::irqNums[0], nvicPriority);
   }

   /**
    * Disable interrupts in NVIC
    */
   static void disableNvicInterrupts() {
      NVIC_DisableIRQ(Info::irqNums[0]);
   }

   /**
    * Converts time in seconds to time in ticks
    *
    * @param[in] seconds Time interval in seconds
    *
    * @return Time in ticks
    *
    * @note This uses the current PDB clock settings (pdb_sc_mult, pdb_sc_prescaler)
    */
   static uint32_t convertSecondsToTicks(float seconds) {

      float clockFrequency = Info::getInputClockFrequency();
      int multValue        = (pdb().SC&PDB_SC_MULT_MASK)>>PDB_SC_MULT_SHIFT;
      int prescaleValue    = (pdb().SC&PDB_SC_PRESCALER_MASK)>>PDB_SC_PRESCALER_SHIFT;

      // Multiplier factors for prescale divider
      static const int multFactors[] = {1,10,20,40};
      float clock = clockFrequency/(multFactors[multValue]*(1<<prescaleValue));
      return round(seconds*clock);
   }

   /**
    * Get 'best' dividers for given period.
    * This involves finding the smallest prescaler that allows the PDB period to be set to greater
    * than the given period.\n
    * This produces the highest resolution.\n
    * It is quite possible that other values would be more suitable for a particular application.
    * For example, carefully chosen prescaler may result in less rounding for the needed intermediate
    * points for pulse outputs etc.
    *
    * @param[in]  period          Period in seconds as a float
    * @param[out] multValue       Determined pdb_sc_mult value
    * @param[out] prescaleValue   Determined pdb_sc_prescaler value
    *
    * @return E_NO_ERROR  => success
    * @return E_ERROR     => failed to find suitable values
    */
   static ErrorCode getDividers(float period, uint32_t &multValue, int &prescaleValue) {

      // Multiplier factors for prescale divider
      static const int   multFactors[] = {1,10,20,40};

      float inputClock = Info::getInputClockFrequency();

      // No MOD value found so far
      uint32_t mod = 0;

      // Try each divider multiplier
      for (unsigned trialMultValue=0; trialMultValue<(sizeof(multFactors)/sizeof(multFactors[0])); trialMultValue++) {
         int multfactor = multFactors[trialMultValue];

         // Try prescalers from smallest to largest
         // Find first prescaler for which a suitable modulo exists
         int prescaleFactor=1;
         for (unsigned trialPrescaleValue=0; trialPrescaleValue<=7; trialPrescaleValue++) {
            float clock = inputClock/(multfactor*prescaleFactor);
            uint32_t trialMod = round(period*clock)-1;
//            console.
//               write("multfactor = ").write(multfactor).
//               write(", prescaleFactor = ").write(prescaleFactor).
//               write(", mod = ").write(trialMod).
//               write(", period = ").writeln(period);
            if (trialMod <= 0) {
               // Too short a period
               return E_TOO_SMALL;
            }
            if (trialMod <= 65535) {
               if (trialMod>mod) {
                  // Better value - save
                  prescaleValue = trialPrescaleValue;
                  multValue     = trialMultValue;
                  mod           = trialMod;
               }
               break;
            }
            prescaleFactor <<= 1;
         }
      }
      return setErrorCode((mod==0)?E_ERROR:E_NO_ERROR);
   }

   /**
    * Sets period of main counter to given value.
    *
    * It attempts to get the 'best' dividers for a given period.\n
    * This involves finding the smallest prescaler that allows the PDB period
    * to be set to greater than the given period.\n
    * This produces the highest resolution.\n
    * It is quite possible that other values would be more suitable for a particular application.\n
    * For example, carefully chosen prescalers may result in less rounding for the needed intermediate
    * points for pulse outputs etc.
    *
    * @param[in]  period Period in seconds as a float
    *
    * @return E_NO_ERROR  => success
    * @return E_ERROR     => failed to find suitable values
    *
    * @note This affects pdb_sc_mult, pdb_sc_prescaler, pdb_mod
    */
   static ErrorCode setPeriod(float period) {

      uint32_t mult     = 0;
      int      prescale = 0;

      ErrorCode rc = getDividers(period, mult, prescale);
      if (rc != E_NO_ERROR) {
         return rc;
      }
      pdb().SC  = (pdb().SC&~(PDB_SC_MULT_MASK|PDB_SC_PRESCALER_MASK))|
            PDB_SC_MULT(mult)|PDB_SC_PRESCALER(prescale)|PDB_SC_PDBIF_MASK;

      // Calculate MOD using new MULT and PRESCALER convertSecondsToTicks()
      pdb().MOD = convertSecondsToTicks(period) - 1;

      return E_NO_ERROR;
   }

   /**
    * Set clock dividers
    *
    * @param[in]  pdbPrescale    Clock pre-scale (pdb_sc_mult)
    * @param[in]  pdbMultiplier  Clock pre-scale multiplier (pdb_sc_prescaler)
    */
   static void setClockDividers(PdbPrescale pdbPrescale, PdbMultiplier pdbMultiplier) {

      pdb().SC  = (pdb().SC&~(PDB_SC_MULT_MASK|PDB_SC_PRESCALER_MASK))|pdbPrescale|pdbMultiplier|PDB_SC_PDBIF_MASK;
   }

   /**
    * Set modulo of the PDB counter (in ticks)
    *
    * @param[in] modulo Modulo value for the counter (pdb_mod)
    */
   static void setModuloInTicks(uint16_t modulo) {

      pdb().MOD = modulo;
   }

   /**
    * Set interrupt delay (in ticks)
    *
    * @param[in] delay Delay value (pdb_idly)
    */
   static void setInterruptDelayInTicks(uint16_t delay) {

      pdb().IDLY = delay;
   }

   /**
    * Set interrupt delay (in seconds)
    *
    * @param[in] delay Delay value for the interrupt (pdb_idly)
    */
   static void setInterruptDelay(float delay) {

      pdb().IDLY = convertSecondsToTicks(delay) - 1;
   }

   /**
    * Set trigger source
    *
    * @param[in] pdbTrigger      Trigger source (pdb_sc_trgsel)
    */
   static void setTriggerSource(PdbTrigger pdbTrigger) {

      pdb().SC = (pdb().SC&~PDB_SC_TRGSEL_MASK)|pdbTrigger|PDB_SC_PDBIF_MASK;
   }

   /**
    * Set one-shot or continuous operation
    *
    * @param[in] pdbMode         PDB mode. Controls if the PDB does one sequence or repeats (pdb_sc_cont)
    */
   static void setMode(PdbMode pdbMode=PdbMode_OneShot) {

      pdb().SC = (pdb().SC&~PDB_SC_CONT_MASK)|pdbMode|PDB_SC_PDBIF_MASK;
   }

   /**
    * Trigger PDB sequence (pdb_sc_swtrig, pdb_sc_trigsel)
    */
   static void softwareTrigger() {

      // Set software trigger + do trigger + without clearing interrupt flag
      pdb().SC |= PDB_SC_TRGSEL_MASK|PDB_SC_SWTRIG_MASK|PDB_SC_PDBIF_MASK;
   }

   /**
    * Enable PDB and configures loading of MOD, IDLY, CHnDLYm, DACINTx,and POyDLY from holding registers
    *
    * @param[in]  pdbLoadMode Controls when the registers are loaded. (pdb_sc_ldmod)
    *
    * @note The actual loading time is governed by pdbLoadMode
    * @note isLoadRegistersComplete() may be used to check if the loading has occurred.
    */
   static void configureRegisterLoad(PdbLoadMode pdbLoadMode) {

      pdb().SC = (pdb().SC&~PDB_SC_LDMOD_MASK)|pdbLoadMode|PDB_SC_PDBEN_MASK|PDB_SC_LDOK_MASK|PDB_SC_PDBIF_MASK;
   }

   /**
    * Indicates if loading of MOD, IDLY, CHnDLYm, DACINTx,and POyDLY registers is complete
    *
    * @note The loading is triggered by confirmRegisterLoad()
    */
   static bool isRegisterLoadComplete() {

      return !(pdb().SC & PDB_SC_LDOK_MASK);
   }

#if PDB_CH_COUNT>0
   /**
    * Configures the pretriggers associated with an ADC.
    *
    * Each pretrigger corresponds to an ADC SC1[n] R[n] register pair used in hardware triggered mode i.e.
    * Channel X, Pretrigger Y => adcX_sc1[Y], adc0_r[Y].
    * For example:
    *   Channel 0, Pretrigger 0 => adc0_sc1[0], adc0_r[0]
    *   Channel 0, Pretrigger 1 => adc0_sc1[1], adc0_r[1] etc.
    *
    * This allows multiple different ADC channels to be converted in a sequence.
    *
    * @param adcNum           ADC associated with the pre-trigger (channel)
    * @param pretriggerNum    Pretrigger being modified
    * @param pdbPretrigger    Pretrigger settings
    * @param delay            Delay in ticks - only needed for PdbPretrigger_Delayed
    */
   static void configureAdcPretriggerInTicks (
         unsigned       adcNum,
         unsigned       pretriggerNum,
         PdbPretrigger  pdbPretrigger,
         uint16_t       delay          = 0) {

      usbdm_assert(adcNum<(sizeof(pdb().CH)/sizeof(pdb().CH[0])),                      "Illegal ADC number");
      usbdm_assert(pretriggerNum<(sizeof(pdb().CH[0].DLY)/sizeof(pdb().CH[0].DLY[0])), "Illegal Pretrigger number");

      uint32_t mask      = (PDB_C1_EN(1)|PDB_C1_BB(1)|PDB_C1_TOS(1))<<pretriggerNum;
      pdb().CH[adcNum].C1                 = (pdb().CH[adcNum].C1&~mask)|(pdbPretrigger<<pretriggerNum);
      pdb().CH[adcNum].DLY[pretriggerNum] = delay - 1;
   }

   /**
    * Configures the pretriggers associated with an ADC.
    *
    * Each pretrigger corresponds to an ADC SC1[n] R[n] register pair used in hardware triggered mode i.e.
    * Channel X, Pretrigger Y => adcX_sc1[Y], adc0_r[Y].
    * For example:
    *   Channel 0, Pretrigger 0 => adc0_sc1[0], adc0_r[0]
    *   Channel 0, Pretrigger 1 => adc0_sc1[1], adc0_r[1] etc.
    *
    * This allows multiple different ADC channels to be converted in a sequence.
    *
    * @param adcNum           ADC associated with the pre-trigger (channel)
    * @param pretriggerNum    Pretrigger being modified
    * @param pdbPretrigger    Pretrigger settings
    * @param delay            Delay - only needed for PdbPretrigger_Delayed
    */
   static void configureAdcPretrigger (
         unsigned       adcNum,
         unsigned       pretriggerNum,
         PdbPretrigger  pdbPretrigger,
         float          delay          = 0.0) {

      configureAdcPretriggerInTicks(adcNum, pretriggerNum, pdbPretrigger, convertSecondsToTicks(delay));
   }

   /**
    * Disables all pretriggers associated with an ADC.
    *
    * @param[in] adcNum          ADC to affect
    */
   static void disableAdcPretriggers(unsigned adcNum) {

      usbdm_assert(adcNum<(sizeof(pdb().CH)/sizeof(pdb().CH[0])), "Illegal ADC number");

      pdb().CH[adcNum].C1 = 0;
   }

   /**
    * Disables a pretrigger associated with an ADC.
    *
    * @param[in] adcNum       ADC to affect
    * @param pretriggerNum    Pretrigger being modified
    */
   static void disableAdcPretrigger(
         unsigned       adcNum,
         unsigned       pretriggerNum) {

      usbdm_assert(adcNum<(sizeof(pdb().CH)/sizeof(pdb().CH[0])), "Illegal ADC number");

      uint32_t mask      = (PDB_C1_EN(1)|PDB_C1_BB(1)|PDB_C1_TOS(1))<<pretriggerNum;
      pdb().CH[adcNum].C1 &= mask;
   }

   /**
    * Get error and sequence flags for the PDB channel (ADC trigger)
    *
    * @param[in] adcNum The ADC to get flags for.
    */
   static uint32_t getChannelFlags(unsigned adcNum) {

      return pdb().CH[adcNum].S;
   }

   /**
    * Clear error and sequence flags in the PDB channel (ADC trigger)
    *
    * @param[in] adcNum The ADC to clear flags for
    */
   static void clearErrorFlags(unsigned adcNum) {

      // Clear flags
      pdb().CH[adcNum].S = 0; // w0c bits
   }

   /**
    * @tparam adcNum The number of the ADC pretrigger (channel) to control
    */
   template<unsigned adcNum>
   class AdcPreTrigger {

      static_assert(adcNum<(sizeof(pdb().CH)/sizeof(pdb().CH[0])), "Illegal ADC number");

   public:
      static constexpr unsigned ADC_NUM = adcNum;

      /**
       * Configures the pretriggers associated with an ADC.
       *
       * Each pretrigger corresponds to an ADC SC1[n] R[n] register pair used in hardware triggered mode i.e.
       * Channel X, Pretrigger Y => adcX_sc1[Y], adc0_r[Y].
       * For example:
       *   Channel 0, Pretrigger 0 => adc0_sc1[0], adc0_r[0]
       *   Channel 0, Pretrigger 1 => adc0_sc1[1], adc0_r[1] etc.
       *
       * This allows multiple different ADC channels to be converted in a sequence.
       *
       * @param pretriggerNum    Pretrigger being modified
       * @param pdbPretrigger    Pretrigger settings
       * @param delay            Delay in ticks - only needed for PdbPretrigger_Delayed
       */
      static void configureInTicks (
            unsigned       pretriggerNum,
            PdbPretrigger  pdbPretrigger,
            uint16_t       delay          = 0) {

         PdbBase_T::configureAdcPretriggerInTicks(adcNum, pretriggerNum, pdbPretrigger, delay);
      }

      /**
       * Configures the pretriggers associated with an ADC.
       *
       * Each pretrigger corresponds to an ADC SC1[n] R[n] register pair used in hardware triggered mode i.e.
       * Channel X, Pretrigger Y => adcX_sc1[Y], adc0_r[Y].
       * For example:
       *   Channel 0, Pretrigger 0 => adc0_sc1[0], adc0_r[0]
       *   Channel 0, Pretrigger 1 => adc0_sc1[1], adc0_r[1] etc.
       *
       * This allows multiple different ADC channels to be converted in a sequence.
       *
       * @param pretriggerNum    Pretrigger being modified
       * @param pdbPretrigger    Pretrigger settings
       * @param delay            Delay - only needed for PdbPretrigger_Delayed
       */
      static void configure (
            unsigned       pretriggerNum,
            PdbPretrigger  pdbPretrigger,
            float          delay          = 0.0) {

         PdbBase_T::configureAdcPretriggerInTicks(adcNum, pretriggerNum, pdbPretrigger, convertSecondsToTicks(delay));
      }

      /**
       * Disables the pretriggers associated with an ADC.
       */
      static void disable() {

         PdbBase_T::disableAdcPretriggers(adcNum);
      }

      /**
       * Get error and sequence flags for the adcNum
       */
      static uint32_t getFlags() {

         return PdbBase_T::getChannelFlags(adcNum);
      }

      /**
       * Clear error and sequence flags for the adcNum
       */
      static void clearFlags() {

         PdbBase_T::clearErrorFlags(adcNum);
      }

   };
#endif

#if PDB_DAC_COUNT>0
   /**
    * DAC Trigger Control
    *
    * There may be multiple DAC triggers generated if the period is smaller that the main counter period.\n
    * The trigger may be bypassed when using an external trigger.
    *
    * @param dacNum            DAC number
    * @param pdbDacTriggerMode Control how the DAC trigger is generated
    * @param period            DAC period in ticks
    */
   static void configureDacTriggerInTicks (
         unsigned          dacNum,
         PdbDacTriggerMode pdbDacTriggerMode,
         uint16_t          period = 0) {

      usbdm_assert(dacNum<(sizeof(pdb().DAC)/sizeof(pdb().DAC[0])), "Illegal DAC number");

      usbdm_assert(
            (pdbDacTriggerMode != PdbDacTriggerMode_External) || (period == 0),
            "DAC period may not be used with external trigger");

      pdb().DAC[dacNum].INTC = pdbDacTriggerMode;
      pdb().DAC[dacNum].INT  = period - 1;
   }

   /**
    * DAC Trigger Control.
    *
    * There may be multiple DAC triggers generated if the period is smaller that the main counter period.\n
    * The trigger may be bypassed when using an external trigger.
    *
    * @param dacNum            DAC number
    * @param pdbDacTriggerMode Controls how the DAC trigger is generated
    * @param period            Interval used to calculate the reload value for DAC interval counter
    */
   static void configureDacTrigger(
         unsigned          dacNum,
         PdbDacTriggerMode pdbDacTriggerMode,
         float             period) {

      configureDacTriggerInTicks(dacNum, pdbDacTriggerMode, convertSecondsToTicks(period));
   }

   /**
    * Disable DAC Trigger associated with a DAC.
    *
    * @param dacNum            DAC number
    */
   static void disableDacTrigger(unsigned dacNum) {

      usbdm_assert(dacNum<(sizeof(pdb().DAC)/sizeof(pdb().DAC[0])), "Illegal DAC number");

      pdb().DAC[dacNum].INTC = 0;
   }

   /**
    * @tparam dacNum The number of the DAC trigger to control
    */
   template<unsigned dacNum>
   class DacTrigger {

      static_assert(dacNum<(sizeof(pdb().DAC)/sizeof(pdb().DAC[0])), "Illegal DAC number");

   public:
      static constexpr unsigned DAC_NUM = dacNum;

      /**
       * DAC Trigger Control
       *
       * There may be multiple DAC triggers generated if the period is smaller that the main counter period.\n
       * The trigger may be bypassed when using an external trigger.
       *
       * @param pdbDacTriggerMode Control how the DAC trigger is generated
       * @param period            Reload value for DAC interval counter
       */
      static void configureInTicks (
            PdbDacTriggerMode pdbDacTriggerMode,
            uint16_t          period = 0) {

         PdbBase_T::configureDacTriggerInTicks(dacNum, pdbDacTriggerMode, period);
      }

      /**
       * DAC Trigger Control
       *
       * There may be multiple DAC triggers generated if the period is smaller that the main counter period.\n
       * The trigger may be bypassed when using an external trigger.
       *
       * @param pdbDacTriggerMode Control how the DAC trigger is generated
       * @param period           Interval used to calculate the reload value for DAC interval counter
       */
      static void configure(
            PdbDacTriggerMode pdbDacTriggerMode,
            float             period) {

         PdbBase_T::configureDacTrigger(dacNum, pdbDacTriggerMode, period);
      }

      /**
       * Disable DAC Trigger Control
       */
      static void disable() {

         PdbBase_T::disableDacTrigger(dacNum);
      }

   };
#endif

#if PDB_POnDLY_COUNT>0
   /**
    * Configure pulse outputs.
    * The pulse outputs are usually associated with comparators.
    *
    * @param outputNum        Pulse output number
    * @param pulseHighDelay   Delay in ticks to start of pulse output
    * @param pulseLowDelay    Delay in ticks to end of pulse output
    */
   static void configurePulseOutputInTicks(
         unsigned outputNum,
         uint16_t pulseHighDelay,
         uint16_t pulseLowDelay) {

      usbdm_assert(outputNum < (sizeof(pdb().POnDLY)/sizeof(pdb().POnDLY[0])), "Illegal pulse output");

      pdb().POEN |= (1<<outputNum);
      pdb().POnDLY[outputNum].DLY1 = pulseHighDelay;
      pdb().POnDLY[outputNum].DLY2 = pulseLowDelay;
   }

   /**
    * Configure pulse outputs.
    * The pulse outputs are usually associated with comparators.
    *
    * @param outputNum        Pulse output number
    * @param pulseHighDelay  Delay in ticks to start of pulse output
    * @param pulseLowDelay    Delay in ticks to end of pulse output
    */
   static void configurePulseOutput(
         unsigned outputNum,
         float    pulseHighDelay,
         float    pulseLowDelay) {

      configurePulseOutputInTicks(outputNum,
            convertSecondsToTicks(pulseHighDelay),
            convertSecondsToTicks(pulseLowDelay));
   }

   /**
    * Disable a pulse output.
    *
    * @param outputNum        Pulse output number
    */
   static void disablePulseOutput(unsigned outputNum) {

      usbdm_assert((1<<outputNum) <= PDB_POEN_POEN_MASK, "Illegal pulse output");
      pdb().POEN &= ~(1<<outputNum);
   }

   /**
    * @tparam outputNum Pulse output number.  Usually corresponds to a comparator.
    */
   template<unsigned outputNum>
   class CmpPulseOutput {

      static_assert(outputNum < (sizeof(pdb().POnDLY)/sizeof(pdb().POnDLY[0])), "Illegal pulse output");

   public:
      static constexpr unsigned PULSE_NUM = outputNum;

      /**
       * Configure pulse outputs.
       * The pulse outputs are usually associated with comparators.
       *
       * @param pulseHighDelay   Delay in ticks to start of pulse output
       * @param pulseLowDelay    Delay in ticks to end of pulse output
       */
      static void configureInTicks(
            uint16_t pulseHighDelay,
            uint16_t pulseLowDelay) {

         PdbBase_T::configurePulseOutputInTicks(outputNum, pulseHighDelay, pulseLowDelay);
      }

      /**
       * Configure pulse outputs.
       * The pulse outputs are usually associated with comparators.
       *
       * @param pulseHighDelay   Delay in ticks to start of pulse output
       * @param pulseLowDelay    Delay in ticks to end of pulse output
       */
      static void configure(
            float    pulseHighDelay,
            float    pulseLowDelay) {

         PdbBase_T::configurePulseOutput(outputNum, pulseHighDelay, pulseLowDelay);
      }

      /**
       * Disable pulse output
       */
      static void disable() {
         pdb().POEN &= ~(1<<outputNum);
      }
   };
#endif
};

template<class Info> PDBCallbackFunction PdbBase_T<Info>::sCallback      = PdbBase_T<Info>::unhandledCallback;
template<class Info> PDBCallbackFunction PdbBase_T<Info>::sErrorCallback = PdbBase_T<Info>::unhandledCallback;

#ifdef USBDM_PDB_IS_DEFINED
/**
 * Class representing PDB
 */
class Pdb : public PdbBase_T<PdbInfo> {};

#endif

#ifdef USBDM_PDB0_IS_DEFINED
/**
 * Class representing PDB
 */
class Pdb0 : public PdbBase_T<Pdb0Info> {};

#endif

#ifdef USBDM_PDB1_IS_DEFINED
/**
 * Class representing PDB
 */
class Pdb1 : public PdbBase_T<Pdb1Info> {};

#endif

/**
 * End PDB_Group
 * @}
 */
} // End namespace USBDM

#endif /* HEADER_PDB_H */

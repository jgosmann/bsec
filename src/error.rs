//! Error types

use libalgobsec_sys::*;
use std::fmt::{self, Debug, Display, Formatter};

/// bsec crate errors.
#[derive(Clone, Debug)]
pub enum Error<E: Debug> {
    /// An variable length vector argument was too long.
    ///
    /// Bosch BSEC only supports up to 256 elements in many places.
    ArgumentListTooLong,
    /// The instance of the BSEC library has already been acquired.
    ///
    /// Only a single BSEC instance can be used per application.
    BsecAlreadyInUse,
    /// An error reported by the Bosch BSEC library.
    BsecError(BsecError),
    /// An error converting data between the bsec crate and the underlying
    /// Bosch BSEC library.
    ConversionError(ConversionError),
    /// An error caused by the BME sensor.
    BmeSensorError(E),
}

impl<E: Debug> Display for Error<E> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), fmt::Error> {
        use Error::*;
        match self {
            ArgumentListTooLong => f.write_str("argument list to BSEC too long"),
            BsecAlreadyInUse => f.write_str("the BSEC instances has already been acquired"),
            BsecError(err) => f.write_fmt(format_args!("BSEC library error: {}", err)),
            ConversionError(err) => {
                f.write_fmt(format_args!("unexpected BSEC return value: {}", err))
            }
            BmeSensorError(err) => f.write_fmt(format_args!(
                "communication failure with BME sensor: {:?}",
                err
            )),
        }
    }
}

impl<E: Debug> std::error::Error for Error<E> {}

impl<E: Debug> From<BsecError> for Error<E> {
    fn from(bsec_error: BsecError) -> Self {
        Self::BsecError(bsec_error)
    }
}

impl<E: Debug> From<ConversionError> for Error<E> {
    fn from(conversion_error: ConversionError) -> Self {
        Self::ConversionError(conversion_error)
    }
}

/// An error converting data between the bsec crate and the underlying Bosch
/// BSEC library.
#[derive(Clone, Debug)]
pub enum ConversionError {
    /// The sample rate was invalid.
    ///
    /// The Bosch BSEC library only supports specific sample rate values.
    /// See the Bosch BSEC documentation.
    InvalidSampleRate(f64),
    /// The physical sensor ID was invalid.
    InvalidPhysicalSensorId(bsec_physical_sensor_t),
    /// The virtual sensor ID was invalid.
    InvalidVirtualSensorId(bsec_virtual_sensor_t),
    /// The accuracy value was invalid.
    ///
    /// The Bosch BSEC library should on report specific accuracy values.
    /// See the Bosch BSEC documentation.
    InvalidAccuracy(u8),
}

impl Display for ConversionError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        use ConversionError::*;
        match self {
            InvalidSampleRate(x) => f.write_fmt(format_args!("invalid sample rate: {}", x)),
            InvalidPhysicalSensorId(x) => {
                f.write_fmt(format_args!("invalid physical sensor ID: {}", x))
            }
            InvalidVirtualSensorId(x) => {
                f.write_fmt(format_args!("invalid virtual sensor ID: {}", x))
            }
            InvalidAccuracy(x) => f.write_fmt(format_args!("invalid accuracy: {}", x)),
        }
    }
}

impl std::error::Error for ConversionError {}

/// Error reported by the Bosch BSEC library.
///
/// See Bosch BSEC documentation.
#[derive(Clone, Debug)]
pub enum BsecError {
    DoStepsInvalidInput,
    DoStepsValueLimits,
    DoStepsDuplicateInput,
    DoStepsNoOutputsReturnable,
    DoStepsExcessOutputs,
    DoStepsTsIntraDiffOutOfRange,
    UpdateSubscriptionWrongDataRate,
    UpdateSubscriptionSampleRateLimits,
    UpdateSubscriptionDuplicateGate,
    UpdateSubscriptionInvalidSampleRate,
    UpdateSubscriptionGateCountExceedsArray,
    UpdateSubscriptionSampleIntervalIntegerMult,
    UpdateSubscriptionMultGaaSamplInterval,
    UpdateSubscriptionHighHeaterOnDuration,
    UpdateSubscriptionUnkownOutputGate,
    UpdateSubscriptionModeInNonUlp,
    UpdateSubscriptionSubscribedOutputGates,
    ParseSectionExceedsWorkBuffer,
    ConfigFail,
    ConfigVersionMismatch,
    ConfigFeatureMismatch,
    ConfigCrcMismatch,
    ConfigEmpty,
    ConfigInsufficientWorkBuffer,
    ConfigInvalidStringSize,
    ConfigInsufficientBuffer,
    SetInvalidChannelIdentifier,
    SetInvalidLength,
    SensorControlCallTimingViolation,
    SensorControlModeExceedsUlpTimelimit,
    SensorControlModeInsufficientWaitTime,
    /// An error not known by the crate.
    Unknown(bsec_library_return_t),
}

impl Display for BsecError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("BSEC error: {:?}", self))
    }
}

impl std::error::Error for BsecError {}

impl From<bsec_library_return_t> for BsecError {
    fn from(return_code: bsec_library_return_t) -> Self {
        #![allow(non_upper_case_globals)]
        use BsecError::*;
        match return_code {
            bsec_library_return_t_BSEC_E_DOSTEPS_INVALIDINPUT => DoStepsInvalidInput,
            bsec_library_return_t_BSEC_E_DOSTEPS_VALUELIMITS => DoStepsValueLimits,
            bsec_library_return_t_BSEC_E_DOSTEPS_DUPLICATEINPUT => DoStepsDuplicateInput,
            bsec_library_return_t_BSEC_I_DOSTEPS_NOOUTPUTSRETURNABLE => DoStepsNoOutputsReturnable,
            bsec_library_return_t_BSEC_W_DOSTEPS_EXCESSOUTPUTS => DoStepsExcessOutputs,
            bsec_library_return_t_BSEC_W_DOSTEPS_TSINTRADIFFOUTOFRANGE => {
                DoStepsTsIntraDiffOutOfRange
            }
            bsec_library_return_t_BSEC_E_SU_WRONGDATARATE => UpdateSubscriptionWrongDataRate,
            bsec_library_return_t_BSEC_E_SU_SAMPLERATELIMITS => UpdateSubscriptionSampleRateLimits,
            bsec_library_return_t_BSEC_E_SU_DUPLICATEGATE => UpdateSubscriptionDuplicateGate,
            bsec_library_return_t_BSEC_E_SU_INVALIDSAMPLERATE => {
                UpdateSubscriptionInvalidSampleRate
            }
            bsec_library_return_t_BSEC_E_SU_GATECOUNTEXCEEDSARRAY => {
                UpdateSubscriptionGateCountExceedsArray
            }
            bsec_library_return_t_BSEC_E_SU_SAMPLINTVLINTEGERMULT => {
                UpdateSubscriptionSampleIntervalIntegerMult
            }
            bsec_library_return_t_BSEC_E_SU_MULTGASSAMPLINTVL => {
                UpdateSubscriptionMultGaaSamplInterval
            }
            bsec_library_return_t_BSEC_E_SU_HIGHHEATERONDURATION => {
                UpdateSubscriptionHighHeaterOnDuration
            }
            bsec_library_return_t_BSEC_W_SU_UNKNOWNOUTPUTGATE => UpdateSubscriptionUnkownOutputGate,
            bsec_library_return_t_BSEC_W_SU_MODINNOULP => UpdateSubscriptionModeInNonUlp,
            bsec_library_return_t_BSEC_I_SU_SUBSCRIBEDOUTPUTGATES => {
                UpdateSubscriptionSubscribedOutputGates
            }
            bsec_library_return_t_BSEC_E_PARSE_SECTIONEXCEEDSWORKBUFFER => {
                ParseSectionExceedsWorkBuffer
            }
            bsec_library_return_t_BSEC_E_CONFIG_FAIL => ConfigFail,
            bsec_library_return_t_BSEC_E_CONFIG_VERSIONMISMATCH => ConfigVersionMismatch,
            bsec_library_return_t_BSEC_E_CONFIG_FEATUREMISMATCH => ConfigFeatureMismatch,
            bsec_library_return_t_BSEC_E_CONFIG_CRCMISMATCH => ConfigCrcMismatch,
            bsec_library_return_t_BSEC_E_CONFIG_EMPTY => ConfigEmpty,
            bsec_library_return_t_BSEC_E_CONFIG_INSUFFICIENTWORKBUFFER => {
                ConfigInsufficientWorkBuffer
            }
            bsec_library_return_t_BSEC_E_CONFIG_INVALIDSTRINGSIZE => ConfigInvalidStringSize,
            bsec_library_return_t_BSEC_E_CONFIG_INSUFFICIENTBUFFER => ConfigInsufficientBuffer,
            bsec_library_return_t_BSEC_E_SET_INVALIDCHANNELIDENTIFIER => {
                SetInvalidChannelIdentifier
            }
            bsec_library_return_t_BSEC_E_SET_INVALIDLENGTH => SetInvalidLength,
            bsec_library_return_t_BSEC_W_SC_CALL_TIMING_VIOLATION => {
                SensorControlCallTimingViolation
            }
            bsec_library_return_t_BSEC_W_SC_MODEXCEEDULPTIMELIMIT => {
                SensorControlModeExceedsUlpTimelimit
            }
            bsec_library_return_t_BSEC_W_SC_MODINSUFFICIENTWAITTIME => {
                SensorControlModeInsufficientWaitTime
            }
            return_code => Unknown(return_code),
        }
    }
}

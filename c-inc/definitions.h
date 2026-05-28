// set the specific model of the chip
#ifndef EFR32MG22E224F512IM40
#define EFR32MG22E224F512IM40 1
#endif

// used for trustzone startup logic
// idk why this is needed as we don't use Trustzone?
// but it's required for compilation
#ifndef __ARM_FEATURE_CMSE_OVERRIDEN

#undef __ARM_FEATURE_CMSE
#define __ARM_FEATURE_CMSE 3U

#define __ARM_FEATURE_CMSE_OVERRIDEN 1

#endif

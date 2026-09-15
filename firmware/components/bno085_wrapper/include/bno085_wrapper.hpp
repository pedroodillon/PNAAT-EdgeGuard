#ifndef COMPONENT_BNO085_WRAPPER_H
#define COMPONENT_BNO085_WRAPPER_H

#include <esp_err.h>

#include "bno085.h"

namespace bno085 {
	esp_err_t init();
	void run_service();
}; // namespace bno085

#endif // COMPONENT_BNO085_WRAPPER_H

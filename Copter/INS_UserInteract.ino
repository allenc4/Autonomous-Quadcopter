#include "INS_UserInteract.h"

bool INS_UserInteract::blocking_read() {
	while (hal.console->available() <= 0) {
		hal.scheduler->delay(20);
	}
	return hal.console->read() != 0;
}

void INS_UserInteract::_printf_P(const prog_char *str, ...) {
	va_list args;
	va_start(args, str);

	hal.console->vprintf_P(str, args);
	va_end(args);
}

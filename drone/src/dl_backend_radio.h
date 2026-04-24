/* dl_backend_radio.h — TX power via iw. */
#pragma once

#include "dl_config.h"
#include "dl_wire.h"

typedef struct dl_backend_radio dl_backend_radio_t;

dl_backend_radio_t *dl_backend_radio_open(const dl_config_t *cfg);
void dl_backend_radio_close(dl_backend_radio_t *br);

/* Apply TX power change if different from `prev`. Updates `prev`. */
int dl_backend_radio_apply(dl_backend_radio_t *br,
                           const dl_decision_t *d,
                           dl_decision_t *prev);

int dl_backend_radio_apply_safe(dl_backend_radio_t *br,
                                const dl_config_t *cfg);

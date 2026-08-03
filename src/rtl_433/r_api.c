/* reduced r_api.c for rtl_433_Decoder_ESP -- from... */

/** @file
    Generic RF data receiver and decoder for ISM band devices using RTL-SDR and
   SoapySDR.

    Copyright (C) 2019 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "r_api.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "pulse_slicer.h"
#include "r_device.h"
#include "r_private.h"
#include "r_util.h"
#include "rtl_433.h"
#include "rtl_433_devices.h"
#include "data.h"
#include "fatal.h"
#include "list.h"
#include "logger.h"
#include "output_log.h"
#include "log.h"

char const* version_string(void) {
  return "rtl_433_Decoder_ESP version 0";
}


/* general */
void r_init_cfg(r_cfg_t* cfg) {

  list_ensure_size(&cfg->output_handler, 1);

  cfg->demod = calloc(1, sizeof(*cfg->demod));
  if (!cfg->demod)
    FATAL_CALLOC("r_init_cfg()");

}

r_cfg_t* r_create_cfg(void) {
  r_cfg_t* cfg = calloc(1, sizeof(*cfg));
  if (!cfg)
    FATAL_CALLOC("r_create_cfg()");

  r_init_cfg(cfg);

  return cfg;
}


/* device decoder protocols */

void register_protocol(r_cfg_t* cfg, r_device* r_dev, char* arg) {
  // use arg of 'v', 'vv', 'vvv' as device verbosity
  int dev_verbose = 0;
  if (arg && *arg == 'v') {
    for (; *arg == 'v'; ++arg) {
      dev_verbose++;
    }
    if (*arg) {
      arg++; // skip separator
    }
  }

  // use any other arg as device parameter
  r_device* p;
  if (r_dev->create_fn) {
    p = r_dev->create_fn(arg);
  } else {
    if (arg && *arg) {
      fprintf(stderr, "Protocol [%u] \"%s\" does not take arguments \"%s\"!\n",
              r_dev->protocol_num, r_dev->name, arg);
    }
    p = malloc(sizeof(*p));
    if (!p)
      FATAL_CALLOC("register_protocol()");
    *p = *r_dev; // copy
  }

  p->verbose = dev_verbose ? dev_verbose : (cfg->verbosity > 4 ? cfg->verbosity - 5 : 0);

  p->log_fn = log_device_handler;

  p->output_fn = data_acquired_handler;
  p->output_ctx = cfg;

  list_push(&cfg->demod->r_devs, p);

  if (cfg->verbosity >= LOG_INFO) {
    fprintf(stderr, "Registering protocol [%u] \"%s\"\n", r_dev->protocol_num,
            r_dev->name);
  }
}

int run_ook_demods(list_t* r_devs, pulse_data_t* pulse_data) {
  int p_events = 0;

  unsigned next_priority = 0; // next smallest on each loop through decoders
  // run all decoders of each priority, stop if an event is produced
  for (unsigned priority = 0; !p_events && priority < UINT_MAX;
       priority = next_priority) {
    next_priority = UINT_MAX;
    for (void** iter = r_devs->elems; iter && *iter; ++iter) {
      r_device* r_dev = *iter;

      // Find next smallest priority
      if (r_dev->priority > priority && r_dev->priority < next_priority)
        next_priority = r_dev->priority;
      // Run only current priority
      if (r_dev->priority != priority)
        continue;

      switch (r_dev->modulation) {
        case OOK_PULSE_PCM:
          // case OOK_PULSE_RZ:
          p_events += pulse_slicer_pcm(pulse_data, r_dev);
          break;
        case OOK_PULSE_PPM:
          p_events += pulse_slicer_ppm(pulse_data, r_dev);
          break;
        case OOK_PULSE_PWM:
          p_events += pulse_slicer_pwm(pulse_data, r_dev);
          break;
        case OOK_PULSE_MANCHESTER_ZEROBIT:
          p_events += pulse_slicer_manchester_zerobit(pulse_data, r_dev);
          break;
        case OOK_PULSE_PIWM_RAW:
          p_events += pulse_slicer_piwm_raw(pulse_data, r_dev);
          break;
        case OOK_PULSE_PIWM_DC:
          p_events += pulse_slicer_piwm_dc(pulse_data, r_dev);
          break;
        case OOK_PULSE_DMC:
          p_events += pulse_slicer_dmc(pulse_data, r_dev);
          break;
        case OOK_PULSE_PWM_OSV1:
          p_events += pulse_slicer_osv1(pulse_data, r_dev);
          break;
        case OOK_PULSE_NRZS:
          p_events += pulse_slicer_nrzs(pulse_data, r_dev);
          break;
        // FSK decoders
        case FSK_PULSE_PCM:
        case FSK_PULSE_PWM:
        case FSK_PULSE_MANCHESTER_ZEROBIT:
          break;
        default:
          fprintf(stderr, "Unknown modulation %u in protocol!\n",
                  r_dev->modulation);
      }
    }
  }

  return p_events;
}

int run_fsk_demods(list_t* r_devs, pulse_data_t* fsk_pulse_data) {
  int p_events = 0;

  unsigned next_priority = 0; // next smallest on each loop through decoders
  // run all decoders of each priority, stop if an event is produced
  for (unsigned priority = 0; !p_events && priority < UINT_MAX;
       priority = next_priority) {
    next_priority = UINT_MAX;
    for (void** iter = r_devs->elems; iter && *iter; ++iter) {
      r_device* r_dev = *iter;

      // Find next smallest priority
      if (r_dev->priority > priority && r_dev->priority < next_priority)
        next_priority = r_dev->priority;
      // Run only current priority
      if (r_dev->priority != priority)
        continue;

      switch (r_dev->modulation) {
        // OOK decoders
        case OOK_PULSE_PCM:
        // case OOK_PULSE_RZ:
        case OOK_PULSE_PPM:
        case OOK_PULSE_PWM:
        case OOK_PULSE_MANCHESTER_ZEROBIT:
        case OOK_PULSE_PIWM_RAW:
        case OOK_PULSE_PIWM_DC:
        case OOK_PULSE_DMC:
        case OOK_PULSE_PWM_OSV1:
        case OOK_PULSE_NRZS:
          break;
        case FSK_PULSE_PCM:
          p_events += pulse_slicer_pcm(fsk_pulse_data, r_dev);
          break;
        case FSK_PULSE_PWM:
          p_events += pulse_slicer_pwm(fsk_pulse_data, r_dev);
          break;
        case FSK_PULSE_MANCHESTER_ZEROBIT:
          p_events += pulse_slicer_manchester_zerobit(fsk_pulse_data, r_dev);
          break;
        default:
          fprintf(stderr, "Unknown modulation %u in protocol!\n",
                  r_dev->modulation);
      }
    }
  }

  return p_events;
}

/* handlers */
/** Pass the data structure to all output handlers. Frees data afterwards. */

void log_device_handler(r_device* r_dev, int level, data_t* data) {
  r_cfg_t* cfg = r_dev->output_ctx;

  for (size_t i = 0; i < cfg->output_handler.len;
       ++i) { // list might contain NULLs
    data_output_t* output = cfg->output_handler.elems[i];
    if (output && output->log_level >= level) {
      data_output_print(output, data);
    }
  }
  data_free(data);
}

/** Pass the data structure to all output handlers. Frees data afterwards. */
void data_acquired_handler(r_device* r_dev, data_t* data) {
  r_cfg_t* cfg = r_dev->output_ctx;

#ifndef NDEBUG
  // check for undeclared csv fields
  for (data_t* d = data; d; d = d->next) {
    int found = 0;
    for (char const* const* p = r_dev->fields; *p; ++p) {
      if (!strcmp(d->key, *p)) {
        found = 1;
        break;
      }
    }
    if (!found) {
      fprintf(stderr, "WARNING: Undeclared field \"%s\" in [%u] \"%s\"\n",
              d->key, r_dev->protocol_num, r_dev->name);
    }
  }
#endif

  if (cfg->conversion_mode == CONVERT_SI) {
    for (data_t* d = data; d; d = d->next) {
      // Convert double type fields ending in _F to _C
      if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_F")) {
        d->value.v_dbl = fahrenheit2celsius(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_F", "_C");
        free(d->key);
        d->key = new_label;
        char* pos;
        if (d->format && (pos = strrchr(d->format, 'F'))) {
          *pos = 'C';
        }
      }
      // Convert double type fields ending in _mph to _kph
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_mph")) {
        d->value.v_dbl = mph2kmph(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_mph", "_kph");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "mi/h", "km/h");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _mi_h to _km_h
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_mi_h")) {
        d->value.v_dbl = mph2kmph(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_mi_h", "_km_h");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "mi/h", "km/h");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _in to _mm
      else if ((d->type == DATA_DOUBLE) &&
               (str_endswith(d->key, "_in") || str_endswith(d->key, "_inch"))) {
        d->value.v_dbl = inch2mm(d->value.v_dbl);
        // need to free ptr returned from str_replace
        char* new_label1 = str_replace(d->key, "_inch", "_in");
        char* new_label2 = str_replace(new_label1, "_in", "_mm");
        free(new_label1);
        free(d->key);
        d->key = new_label2;
        char* new_format_label = str_replace(d->format, "in", "mm");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _in_h to _mm_h
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_in_h")) {
        d->value.v_dbl = inch2mm(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_in_h", "_mm_h");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "in/h", "mm/h");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _inHg to _hPa
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_inHg")) {
        d->value.v_dbl = inhg2hpa(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_inHg", "_hPa");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "inHg", "hPa");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _PSI to _kPa
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_PSI")) {
        d->value.v_dbl = psi2kpa(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_PSI", "_kPa");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "PSI", "kPa");
        free(d->format);
        d->format = new_format_label;
      }
    }
  }
  if (cfg->conversion_mode == CONVERT_CUSTOMARY) {
    for (data_t* d = data; d; d = d->next) {
      // Convert double type fields ending in _C to _F
      if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_C")) {
        d->value.v_dbl = celsius2fahrenheit(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_C", "_F");
        free(d->key);
        d->key = new_label;
        char* pos;
        if (d->format && (pos = strrchr(d->format, 'C'))) {
          *pos = 'F';
        }
      }
      // Convert double type fields ending in _kph to _mph
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_kph")) {
        d->value.v_dbl = kmph2mph(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_kph", "_mph");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "km/h", "mi/h");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _km_h to _mi_h
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_km_h")) {
        d->value.v_dbl = kmph2mph(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_km_h", "_mi_h");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "km/h", "mi/h");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _mm to _inch
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_mm")) {
        d->value.v_dbl = mm2inch(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_mm", "_in");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "mm", "in");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _mm_h to _in_h
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_mm_h")) {
        d->value.v_dbl = mm2inch(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_mm_h", "_in_h");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "mm/h", "in/h");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _hPa to _inHg
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_hPa")) {
        d->value.v_dbl = hpa2inhg(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_hPa", "_inHg");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "hPa", "inHg");
        free(d->format);
        d->format = new_format_label;
      }
      // Convert double type fields ending in _kPa to _PSI
      else if ((d->type == DATA_DOUBLE) && str_endswith(d->key, "_kPa")) {
        d->value.v_dbl = kpa2psi(d->value.v_dbl);
        char* new_label = str_replace(d->key, "_kPa", "_PSI");
        free(d->key);
        d->key = new_label;
        char* new_format_label = str_replace(d->format, "kPa", "PSI");
        free(d->format);
        d->format = new_format_label;
      }
    }
  }

  //data_append(data, "protocol", "", DATA_STRING, r_dev->name,NULL);
  data = data_str(data, "protocol", "protocol", NULL, r_dev->name);
  
  size_t message_size = 2000; // should be plenty big
  char *message       = (char *) malloc(message_size);
  if (!message) {
      WARN_MALLOC("data_acquired json callback message alloc");
      data_free(data);
      return; // NOTE: skip output on alloc failure.
  }

  data_print_jsons(data, message, message_size);

  // callback to external function that receives message from device (
  // rtl_433_Decoder )
  (cfg->callback)(message,cfg->ctx);
  data_free(data);
}


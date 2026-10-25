/*
 * FpiEnv - Reading settings from the environment
 * Copyright (C) 2026 Hugo Sales <hugo@hsal.es>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "fpi-env.h"

#include <math.h>

/**
 * SECTION:fpi-env
 * @title: Environment settings
 * @short_description: Reading numeric settings from the environment
 *
 * Drivers that let a setting be overridden through an environment
 * variable can read it with these functions. A value that is set but
 * does not parse in full, or falls outside the accepted range, is logged
 * and replaced by the caller's default.
 */

/**
 * fpi_env_get_int64:
 * @name: Name of the environment variable
 * @min: Smallest accepted value
 * @max: Largest accepted value
 * @def: Value to use when @name is unset, empty or invalid
 *
 * Reads a decimal integer from the environment variable @name. An unset
 * or empty variable is not an error and is not logged.
 *
 * Returns: the integer @name holds if it is from @min to @max, @def
 *   otherwise
 */
gint64
fpi_env_get_int64 (const char *name,
                   gint64      min,
                   gint64      max,
                   gint64      def)
{
  const char *value = g_getenv (name);
  gint64 n;

  g_return_val_if_fail (min <= max, def);

  if (!value || !*value)
    return def;

  if (!g_ascii_string_to_signed (value, 10, min, max, &n, NULL))
    {
      g_message ("Ignoring %s=\"%s\", expected an integer from %" G_GINT64_FORMAT
                 " to %" G_GINT64_FORMAT ", using %" G_GINT64_FORMAT,
                 name, value, min, max, def);
      return def;
    }

  return n;
}

/**
 * fpi_env_get_double:
 * @name: Name of the environment variable
 * @min: Smallest accepted value
 * @max: Largest accepted value
 * @def: Value to use when @name is unset, empty or invalid
 *
 * Reads a number from the environment variable @name, which is parsed
 * the same way in every locale. NaN and infinities are never accepted.
 * An unset or empty variable is not an error and is not logged.
 *
 * Returns: the number @name holds if it is from @min to @max, @def
 *   otherwise
 */
double
fpi_env_get_double (const char *name,
                    double      min,
                    double      max,
                    double      def)
{
  const char *value = g_getenv (name);
  char *end;
  double d;

  g_return_val_if_fail (min <= max, def);

  if (!value || !*value)
    return def;

  d = g_ascii_strtod (value, &end);
  if (end == value || *end || !isfinite (d) || d < min || d > max)
    {
      g_message ("Ignoring %s=\"%s\", expected a number from %g to %g, using %g",
                 name, value, min, max, def);
      return def;
    }

  return d;
}

/*
 * Unit tests for reading settings from the environment
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

#define VAR "FP_TEST_ENV_SETTING"

static void
test_int64_unset (void)
{
  g_unsetenv (VAR);
  g_assert_cmpint (fpi_env_get_int64 (VAR, 0, 10, 7), ==, 7);
  g_setenv (VAR, "", TRUE);
  g_assert_cmpint (fpi_env_get_int64 (VAR, 0, 10, 7), ==, 7);
  g_unsetenv (VAR);
}

static void
test_int64_valid (void)
{
  const struct
  {
    const char *value;
    gint64      min, max, want;
  } cases[] = {
    { "0", 0, 10, 0 },
    { "10", 0, 10, 10 },
    { "-3", -5, 5, -3 },
    { "9223372036854775807", 0, G_MAXINT64, G_MAXINT64 },
    { "-9223372036854775808", G_MININT64, 0, G_MININT64 },
  };

  for (guint i = 0; i < G_N_ELEMENTS (cases); i++)
    {
      g_setenv (VAR, cases[i].value, TRUE);
      g_assert_cmpint (fpi_env_get_int64 (VAR, cases[i].min, cases[i].max, 7),
                       ==, cases[i].want);
    }
  g_unsetenv (VAR);
}

static void
test_int64_invalid (void)
{
  const char *values[] = {
    "abc", "5x", "1.5", "0x10", " 5", "11", "-1", "99999999999999999999",
  };

  for (guint i = 0; i < G_N_ELEMENTS (values); i++)
    {
      g_autofree char *msg = g_strdup_printf ("Ignoring " VAR "=\"%s\", "
                                              "expected an integer from 0 to 10, "
                                              "using 7",
                                              values[i]);

      g_setenv (VAR, values[i], TRUE);
      g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_MESSAGE, msg);
      g_assert_cmpint (fpi_env_get_int64 (VAR, 0, 10, 7), ==, 7);
      g_test_assert_expected_messages ();
    }
  g_unsetenv (VAR);
}

static void
test_double_unset (void)
{
  g_unsetenv (VAR);
  g_assert_cmpfloat (fpi_env_get_double (VAR, 0, 1, 0.5), ==, 0.5);
  g_setenv (VAR, "", TRUE);
  g_assert_cmpfloat (fpi_env_get_double (VAR, 0, 1, 0.5), ==, 0.5);
  g_unsetenv (VAR);
}

static void
test_double_valid (void)
{
  const struct
  {
    const char *value;
    double      want;
  } cases[] = {
    { "0", 0 },
    { "1", 1 },
    { "0.25", 0.25 },
    { "-0.75", -0.75 },
    { "1e-1", 0.1 },
  };

  for (guint i = 0; i < G_N_ELEMENTS (cases); i++)
    {
      g_setenv (VAR, cases[i].value, TRUE);
      g_assert_cmpfloat (fpi_env_get_double (VAR, -1, 1, 0.5), ==, cases[i].want);
    }
  g_unsetenv (VAR);
}

static void
test_double_invalid (void)
{
  const char *values[] = {
    "abc", "0.5x", "nan", "inf", "-inf", "1e999", "1.5", "-2",
  };

  for (guint i = 0; i < G_N_ELEMENTS (values); i++)
    {
      g_autofree char *msg = g_strdup_printf ("Ignoring " VAR "=\"%s\", "
                                              "expected a number from -1 to 1, "
                                              "using 0.5",
                                              values[i]);

      g_setenv (VAR, values[i], TRUE);
      g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_MESSAGE, msg);
      g_assert_cmpfloat (fpi_env_get_double (VAR, -1, 1, 0.5), ==, 0.5);
      g_test_assert_expected_messages ();
    }
  g_unsetenv (VAR);
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/env/int64/unset", test_int64_unset);
  g_test_add_func ("/env/int64/valid", test_int64_valid);
  g_test_add_func ("/env/int64/invalid", test_int64_invalid);
  g_test_add_func ("/env/double/unset", test_double_unset);
  g_test_add_func ("/env/double/valid", test_double_valid);
  g_test_add_func ("/env/double/invalid", test_double_invalid);

  return g_test_run ();
}

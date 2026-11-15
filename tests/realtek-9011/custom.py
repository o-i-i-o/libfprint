#!/usr/bin/env python3

import sys
import traceback
import gi

gi.require_version('FPrint', '2.0')
from gi.repository import FPrint, GLib

# Exit with error on any exception, included those happening in async callbacks
sys.excepthook = lambda *args: (traceback.print_exception(*args), sys.exit(1))

ctx = GLib.main_context_default()

c = FPrint.Context()
c.enumerate()
devices = c.get_devices()

d = devices[0]
del devices

assert d.get_driver() == "realtek"
assert d.get_nr_enroll_stages() == 12
assert not d.has_feature(FPrint.DeviceFeature.CAPTURE)
assert d.has_feature(FPrint.DeviceFeature.IDENTIFY)
assert d.has_feature(FPrint.DeviceFeature.VERIFY)
assert not d.has_feature(FPrint.DeviceFeature.DUPLICATES_CHECK)
assert d.has_feature(FPrint.DeviceFeature.STORAGE)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_LIST)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_DELETE)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_CLEAR)

d.open_sync()

d.clear_storage_sync()

# Listing an empty template table must return an empty list without error,
# even when GET_TEMPLATE (0x45 0x0e) reports a non-zero status byte.
empty_stored = d.list_prints_sync()
assert len(empty_stored) == 0
del empty_stored

stages_seen = []


def enroll_progress(device, completed_stages, print_obj, user_data, error):
    assert error is None
    assert device.get_finger_status() == FPrint.FingerStatusFlags.NONE
    stages_seen.append(completed_stages)
    print(f"enroll progress: {completed_stages}/{device.get_nr_enroll_stages()}")


identified = False


def identify_done(dev, res):
    global identified
    identified = True
    identify_match, identify_print = dev.identify_finish(res)
    assert identify_match is not None
    assert identify_print is not None
    assert identify_match.equal(identify_print)


template = FPrint.Print.new(d)

print("enrolling")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
p = d.enroll_sync(template, None, enroll_progress, None)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
assert stages_seen == list(range(1, 13))
print("enroll done")

print("listing")
stored = d.list_prints_sync()
assert len(stored) == 1
assert stored[0].equal(p)
print("listing done")

print("verifying")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
verify_res, verify_print = d.verify_sync(p)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
assert verify_res is True
assert verify_print.equal(p)
del verify_print
print("verify done")

deserialized_prints = []
for s in stored:
    deserialized_prints.append(FPrint.Print.deserialize(s.serialize()))
    assert deserialized_prints[-1].equal(s)
del stored

print("async identifying")
d.identify(deserialized_prints, callback=identify_done)
del deserialized_prints

while not identified:
    ctx.iteration(True)

print("deleting")
d.delete_print_sync(p)
del p
print("delete done")

d.close_sync()

del d
del c

#!/usr/bin/env python3

import traceback
import sys
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

assert d.get_driver() == "petaicmoc"
assert not d.has_feature(FPrint.DeviceFeature.CAPTURE)
assert d.has_feature(FPrint.DeviceFeature.IDENTIFY)
assert d.has_feature(FPrint.DeviceFeature.VERIFY)
assert not d.has_feature(FPrint.DeviceFeature.DUPLICATES_CHECK)
assert d.has_feature(FPrint.DeviceFeature.STORAGE)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_LIST)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_DELETE)
# Clearing the storage would also wipe templates that other software keeps in the sensor
assert not d.has_feature(FPrint.DeviceFeature.STORAGE_CLEAR)

d.open_sync()

template = FPrint.Print.new(d)
template.set_finger(FPrint.Finger.RIGHT_INDEX)

def enroll_progress(*args):
    assert d.get_finger_status() & FPrint.FingerStatusFlags.NEEDED
    print('enroll progress: ' + str(args))

def identify_done(dev, res):
    global identified
    identified = True
    identify_match, identify_print = dev.identify_finish(res)
    print('indentification_done: ', identify_match, identify_print)
    assert identify_match.equal(identify_print)

# Enroll, list, verify, identify, delete, list
print("enrolling")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
p = d.enroll_sync(template, None, enroll_progress, None)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
print("enroll done")
assert p.get_finger() == FPrint.Finger.RIGHT_INDEX

# The sensor can hold templates of other software, only our own ones are listed
print("listing")
stored = d.list_prints_sync()
print("listing done")
assert any(s.equal(p) for s in stored)

print("verifying")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
verify_res, verify_print = d.verify_sync(p)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
print("verify done")
assert verify_res == True

identified = False
deserialized_prints = []
for s in stored:
    deserialized_prints.append(FPrint.Print.deserialize(s.serialize()))
    assert deserialized_prints[-1].equal(s)
del stored

print('async identifying')
d.identify(deserialized_prints, callback=identify_done)
del deserialized_prints

while not identified:
    ctx.iteration(True)

print("deleting")
d.delete_print_sync(p)
print("delete done")

stored = d.list_prints_sync()
assert not any(s.equal(p) for s in stored)

d.close_sync()

del d
del c

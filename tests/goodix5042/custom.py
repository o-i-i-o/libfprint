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

assert d.get_driver() == "goodix5042"
assert not d.has_feature(FPrint.DeviceFeature.CAPTURE)
assert d.has_feature(FPrint.DeviceFeature.IDENTIFY)
assert d.has_feature(FPrint.DeviceFeature.VERIFY)
assert not d.has_feature(FPrint.DeviceFeature.STORAGE)
assert d.get_nr_enroll_stages() == 15

d.open_sync()

template = FPrint.Print.new(d)

def enroll_progress(*args):
    print('enroll progress: ' + str(args))

def identify_done(dev, res):
    global identified
    identified = True
    identify_match, identify_print = dev.identify_finish(res)
    print('identification done: ', identify_match, identify_print)
    # no print is built from the probe: only the match is reported
    assert identify_match is not None
    assert identify_match.equal(deserialized)

# Enroll, verify, identify (host-side matching: the print carries the
# template)
print("enrolling")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
p = d.enroll_sync(template, None, enroll_progress, None)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
print("enroll done")

print("verifying")
verify_res, verify_print = d.verify_sync(p)
print("verify done")
assert verify_res == True

identified = False
deserialized = FPrint.Print.deserialize(p.serialize())
assert deserialized.equal(p)

print('async identifying')
d.identify([deserialized], callback=identify_done)
while not identified:
    ctx.iteration(True)

d.close_sync()

del d
del c

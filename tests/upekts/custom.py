#!/usr/bin/env python3

from __future__ import annotations

import os

import gi

gi.require_version('FPrint', '2.0')
from gi.repository import FPrint, GLib


def present(message: str) -> None:
    print(message, flush=True)
    if os.getenv('UPEK_INTERACTIVE'):
        input('Press Enter to start this operation.\n')


def enroll_progress(device: FPrint.Device, stage: int,
                    print_: FPrint.Print | None, error: GLib.Error | None,
                    data: object) -> None:
    assert 0 <= stage <= device.get_nr_enroll_stages()
    if error is not None:
        print(f'Enrollment retry: {error.message}', flush=True)
    else:
        print(f'Enrollment stage {stage}; swipe the same finger again.', flush=True)


def verify(enrolled: FPrint.Print) -> bool:
    for attempt in range(5):
        try:
            matched, _ = device.verify_sync(enrolled)
            return matched
        except GLib.Error as error:
            if not error.matches(FPrint.DeviceRetry.quark(), error.code):
                raise
            print(f'Verification retry: {error.message}', flush=True)
    raise AssertionError('No usable swipe after five attempts')


context = FPrint.Context()
context.enumerate()
device = next(d for d in context.get_devices() if d.get_driver() == 'upekts')
assert device.get_nr_enroll_stages() == 3
assert device.has_feature(FPrint.DeviceFeature.VERIFY)
assert not device.has_feature(FPrint.DeviceFeature.STORAGE_LIST)

device.open_sync()
try:
    present('Enroll: swipe the right index finger until enrollment completes.')
    enrolled = device.enroll_sync(FPrint.Print.new(device), None, enroll_progress, None)
    if os.getenv('UMOCKDEV_DIR'):
        assert bytes(enrolled.get_property('fpi-data').unpack()) == b'Z' * 232
    present('Enrollment completed. Match: swipe the right index finger.')
    matched = verify(enrolled)
    assert matched

    present('Match confirmed. Non-match: swipe the left index finger.')
    matched = verify(enrolled)
    assert not matched

    device.close_sync()
    device.open_sync()
    present('Reopened. Match: swipe the right index finger.')
    matched = verify(enrolled)
    assert matched

    try:
        device.list_prints_sync()
    except GLib.Error as error:
        assert error.matches(FPrint.DeviceError.quark(), FPrint.DeviceError.NOT_SUPPORTED)
    else:
        raise AssertionError('upekts must reject listing stored prints')
    print('Enrollment, match, non-match, reopen, and unsupported listing passed.', flush=True)
finally:
    if device.is_open():
        device.close_sync()

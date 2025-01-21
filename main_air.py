#! /usr/bin/env python

from subprocess import call
from threading import Thread

import fly_by_ip_air

Thread(target=fly_by_ip_air.openSerial).start()

call(['./mediamtx'])



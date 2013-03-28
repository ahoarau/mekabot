#! /usr/bin/python

# aseqdump.py -- python port of aplaymidi
# Copyright (C) 2008 Aldrin Martoq <amartoq@dcc.uchile.cl>
#
# Based on code from aseqdump.c, from ALSA project
# Copyright (C) 2005 Clemens Ladisch <clemens@ladisch.de>
#
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA


import sys
sys.path.insert(0, '../pyalsa')

import getopt
import os
import struct
import alsaseq
import traceback
import time
from alsaseq import *

def errormsg(msg, *args):
    """ prints an error message to stderr """
    sys.stderr.write(msg % args)
    sys.stderr.write('\n')
    traceback.print_exc(file=sys.stderr)


def fatal(msg, *args):
    """ prints an error message to stderr, and dies """
    errormsg(msg, *args)
    sys.exit(1)


def init_seq():
    """ opens an alsa sequencer """
    try:
        sequencer = Sequencer(name = 'default',
                              clientname = 'aseqdump.py',
                              streams = SEQ_OPEN_DUPLEX,
                              mode = SEQ_BLOCK)
        return sequencer
    except SequencerError, e:
        fatal("open sequencer: %e", e)


def parse_ports(sequencer, portspec):
    """ parses one or more port addresses from the string, separated by ','
    example: 14:0,Timidity
    """
    portlist = []
    if portspec == None:
        return portlist
    ports = portspec.split(',')
    for port in ports:
        try:
            client, port = sequencer.parse_address(port)
            portlist.append((client, port))
        except SequencerError, e:
            fatal("Failed to parse port %s - %s", port, e)
    return portlist


def create_source_port(sequencer):
    try :
        port = sequencer.create_simple_port(name = 'aseqdump.py',
                                            type = SEQ_PORT_TYPE_MIDI_GENERIC
                                            | SEQ_PORT_TYPE_APPLICATION,
                                            caps = SEQ_PORT_CAP_WRITE
                                            | SEQ_PORT_CAP_SUBS_WRITE)
        return port
    except SequencerError, e:
        fatal("Failed to create port - %s", e)


def connect_ports(sequencer, port_id, ports):
    client_id = sequencer.client_id
    for client, port in ports:
        sequencer.connect_ports((client, port),(client_id, port_id))

def dump_event(event):
    print "%3d:%-3d" % ((event.source)[0], (event.source)[1]),
    type = event.type
    data = event.get_data()
    if type == SEQ_EVENT_NOTEON:
        print "Note on                %2d %3d %3d" % \
            (data['note.channel'], data['note.note'], data['note.velocity'])
    elif type == SEQ_EVENT_NOTEOFF:
        print "Note off               %2d %3d %3d" % \
            (data['note.channel'], data['note.note'], data['note.velocity'])
    elif type == SEQ_EVENT_KEYPRESS:
        print "Polyphonic aftertouch  %2d %3d %3d" % \
            (data['note.channel'], data['note.note'], data['note.velocity'])
    elif type == SEQ_EVENT_CONTROLLER:
        print "Control change         %2d %3d %3d" % \
            (data['control.channel'], data['control.param'], data['control.value'])
    elif type == SEQ_EVENT_PGMCHANGE:
        print "Program change         %2d %3d" % \
            (data['control.channel'], data['control.value'])
    else:
        print "Event type %d" % type

def list_ports():
    sequencer = init_seq()
    print " Port    Client name                      Port name";
    clientports = sequencer.connection_list()
    for connections in clientports:
        clientname, clientid, connectedports = connections
        for port in connectedports:
            portname, portid, connections = port
            portinfo = sequencer.get_port_info(portid, clientid)
            caps = portinfo['capability']
            if caps & (SEQ_PORT_CAP_READ | SEQ_PORT_CAP_SUBS_READ):
                print "%3d:%-3d  %-32.32s %s" % (clientid, portid, clientname, portname)

def usage():
    print "Usage: %s [options]\n" \
        "\nAvailable options:\n" \
        "-h, --help                  this help\n" \
        "-V, --version               show version\n" \
        "-l, --list                  list input ports\n" \
        "-p, --port=client:port,...  source port(s)\n" \
        % (sys.argv[0])

def version():
    print "aseqdump.py asoundlib version %s" % (SEQ_LIB_VERSION_STR)


def main():
    sequencer = init_seq()
    ports = []
    end_delay = 2

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hVlp:", ["help", "version", "list", "port="])
    except getopt.GetoptError:
        usage()
        sys.exit(2)
    for o, a in opts:
        if o in ("-h", "--help"):
            usage()
            sys.exit(0)
        elif o in ("-V", "--version"):
            version()
            sys.exit(0)
        elif o in ("-l", "--list"):
            list_ports()
            sys.exit(0)
        elif o in ("-p", "--port"):
            ports = parse_ports(sequencer, a)
        elif o in ("-d", "--delay"):
            end_delay = int(a)

    source_port = create_source_port(sequencer)
    connect_ports(sequencer, source_port, ports)

    sequencer.mode = SEQ_NONBLOCK

    if len(ports) > 0:
        print "Waiting for data.",
    else:
        print "Waiting for data at port %d:0." % sequencer.client_id,

    print "Press Ctrl+C to end."
    print "Source_ Event_________________ Ch _Data__"

    while True:
        try:
            eventlist = sequencer.receive_events(timeout=10069, maxevents = 1)
            for event in eventlist:
                dump_event(event)
        except KeyboardInterrupt:
            pass
            break



if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import argparse
import sys
from tools.lib.route import Route
from tools.lib.logreader import MultiLogIterator

def main(route_name: str, message_id: int, bus: int):
  # setup a MultiLogIterator to read all the logs in the route
  r = Route(route_name)
  lr = MultiLogIterator(r.log_paths())

  for msg in lr:
    if msg.which() == "can":
      # msg.can is a List of CanData
      # struct CanData {
      #   address @0 :UInt32;
      #   busTime @1 :UInt16;
      #   dat     @2 :Data;
      #   src     @3 :UInt8;
      # }

      for canmsg in msg.can:
        if canmsg.address == message_id and canmsg.src == bus:
          print(canmsg.dat.hex())




if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Utility to dump binary data for a CAN message. Redirect if needed in file",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, help="Specify bus - defaults to 0", default=0)
  parser.add_argument("route_name", help="The route or segment name to filter")
  parser.add_argument("message_id", type=int, help="The CAN msg id to be exported")
  # TODO: Allow multiple message ids, and outputting fields like bustime
  
  if len(sys.argv) == 1:
    parser.print_help()
    sys.exit()
  args = parser.parse_args()
  
  rn = args.route_name.strip()
  main(rn, args.message_id, args.bus)
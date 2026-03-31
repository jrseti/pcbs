import pcbnew
import math
import os

# User parameters
INPUT_FILE = "DishyPi1.kicad_pcb"      # your board file
OUTPUT_FILE = "stitched_output.kicad_pcb"
TARGET_NET_NAMES = ["GNDPI", "GNDIO"]
VIA_HOLE = pcbnew.FromMM(3.0)
VIA_DIAM = pcbnew.FromMM(6.0)
SPACING = pcbnew.FromMM(2.54)
LAYER_TOP = pcbnew.F_Cu
LAYER_GND = pcbnew.In1_Cu  # Replace with your ground layer

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def main():
    print("HELLO")
    board = pcbnew.LoadBoard(INPUT_FILE)
    netinfo = board.GetNetsByName()

    via_count = 0

    for net_name in TARGET_NET_NAMES:
        if net_name not in netinfo:

            print(f"[!] Net '{net_name}' not found.")
            continue

        net = netinfo[net_name]
        zones = [
            z for z in board.Zones()
            #if z.GetNetCode() == net.GetNet()
            if z.GetNetCode() == net.GetNetCode()
            and z.GetLayer() == LAYER_TOP
        ]

        for zone in zones:
            bbox = zone.GetBoundingBox()
            xmin = bbox.GetX()
            xmax = bbox.GetX() + bbox.GetWidth()
            ymin = bbox.GetY()
            ymax = bbox.GetY() + bbox.GetHeight()

            y = ymin
            while y <= ymax:
                x = xmin
                while x <= xmax:
                    pt = pcbnew.wxPoint(x, y)
                    #if zone.IsPointInside(pt):
                    inside = False
                    for poly in zone.GetFilledPolysList():
                        if poly.PointInPolygon(pt):
                            inside = True
                            break

                    if inside:
                        via = pcbnew.VIA(board)
                        via.SetPosition(pt)
                        via.SetViaType(pcbnew.VIA_THROUGH)
                        via.SetDrill(VIA_HOLE)
                        via.SetWidth(VIA_DIAM)
                        via.SetLayerPair(LAYER_TOP, LAYER_GND)
                        via.SetNet(net)
                        board.Add(via)
                        via_count += 1
                    x += SPACING
                y += SPACING

    pcbnew.SaveBoard(OUTPUT_FILE, board)
    print(f"[✓] Done. {via_count} vias inserted.")
    print(f"[✓] Output written to: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
s
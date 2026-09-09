# -*- coding: utf-8 -*-
"""Zamienia zrzut mapy sieci z wezla ("MAP ..." na serialu) w graf.

Uzycie:
    python map_graph.py serial_log.txt          # podsumowanie tekstowe
    python map_graph.py serial_log.txt --dot    # graf w formacie Graphviz
    type COM.log | python map_graph.py -        # ze standardowego wejscia

Wezel wypisuje mape po komendzie "MAP" (a po "MAP <id>" dopytuje odlegly wezel;
odpowiedz dopisuje sie do obrazu jako kolejne linie "MAP E"). Format:

    MAP BEGIN <wezel> <czas pracy s>
    MAP N <sasiad> <tlumienie dB>            lacze zmierzone przez ten wezel
    MAP E <a> <b> <tlumienie dB> <wiek s>    lacze uslyszane od kogos innego
    MAP R <cel> <przez> <koszt>              z czego korzysta routing
    MAP END
    MAP RESP <wezel> <liczba sasiadow>       naglowek odpowiedzi na zapytanie

Brany jest POD UWAGE OSTATNI kompletny zrzut w pliku - log potrafi obejmowac
wiele godzin, a mapa ma opisywac stan biezacy. Linie "MAP E" spoza zrzutu
(odpowiedzi, ktore przyszly pozniej) sa dokladane na koncu.
"""
import sys
import re

# Linie z rejestratora bywaja poprzedzone znacznikiem czasu i prefiksem portu,
# np. "12:31:02 [COM] | MAP E 1 3 108 4" - bierzemy wszystko od slowa MAP.
LINE = re.compile(r"MAP\s+(BEGIN|END|RESP|[NER])\b(.*)")


def parse(text):
    collector = None
    uptime = None
    own = {}      # sasiad -> tlumienie (lacze zmierzone przez kolektor)
    edges = {}    # (a, b) -> (tlumienie, wiek)
    routes = {}   # cel -> (przez, koszt)
    started = False

    for raw in text.splitlines():
        m = LINE.search(raw)
        if not m:
            continue
        kind, rest = m.group(1), m.group(2).split()
        try:
            if kind == "BEGIN":
                # Nowy zrzut uniewaznia poprzedni, ale nie zebrane pozniej odpowiedzi.
                collector, uptime = int(rest[0]), int(rest[1])
                own, routes = {}, {}
                started = True
            elif kind == "N" and started:
                own[int(rest[0])] = int(rest[1])
            elif kind == "E":
                a, b, loss = int(rest[0]), int(rest[1]), int(rest[2])
                age = int(rest[3]) if len(rest) > 3 else 0
                edges[(min(a, b), max(a, b))] = (loss, age)
            elif kind == "R" and started:
                routes[int(rest[0])] = (int(rest[1]), int(rest[2]))
        except (IndexError, ValueError):
            continue  # linia ucieta w polowie przez inny watek logu

    if collector is None:
        return None
    for neighbour, loss in own.items():
        edges[(min(collector, neighbour), max(collector, neighbour))] = (loss, 0)
    return collector, uptime, own, edges, routes


def quality(loss_db):
    """Slowny opis tlumienia - ta sama skala, wedlug ktorej wezel liczy koszt trasy."""
    if loss_db <= 70:
        return "bardzo dobre"
    if loss_db <= 90:
        return "dobre"
    if loss_db <= 110:
        return "slabe"
    return "na granicy"


def as_text(collector, uptime, own, edges, routes):
    out = ["Mapa widziana z wezla %d (czas pracy %d s)" % (collector, uptime), ""]
    nodes = sorted({n for edge in edges for n in edge} | set(routes))
    out.append("Wezly: " + ", ".join(str(n) for n in nodes))
    out.append("")
    out.append("Lacza (tlumienie sciezki):")
    for (a, b), (loss, age) in sorted(edges.items()):
        mine = " zmierzone" if a == collector or b == collector else ""
        stale = "" if age == 0 else ", sprzed %d s" % age
        out.append("  %d - %d   %3d dB  (%s%s)%s" % (a, b, loss, quality(loss), stale, mine))
    if routes:
        out.append("")
        out.append("Trasy uzywane przez wezel %d:" % collector)
        for dest in sorted(routes):
            via, cost = routes[dest]
            how = "bezposrednio" if via == dest else "przez %d" % via
            unreachable = "  NIEOSIAGALNY" if cost >= 255 else ""
            out.append("  do %d: %s (koszt %d)%s" % (dest, how, cost, unreachable))
    unknown = [n for n in nodes if n != collector and n not in own
               and not any(n in edge for edge in edges if collector in edge)]
    if unknown:
        out.append("")
        out.append("Poza zasiegiem beaconow (wiedza z drugiej reki): "
                   + ", ".join(str(n) for n in unknown))
    return "\n".join(out)


def as_dot(collector, edges):
    out = ["graph siec {", '  layout=neato;', '  node [shape=circle];']
    for node in sorted({n for edge in edges for n in edge}):
        mark = ' [style=filled fillcolor=lightgrey]' if node == collector else ''
        out.append('  %d%s;' % (node, mark))
    for (a, b), (loss, _) in sorted(edges.items()):
        # Grubsza krawedz = lepsze lacze; dlugosc rosnie z tlumieniem.
        width = max(1, 5 - (loss - 60) // 15)
        out.append('  %d -- %d [label="%d dB", penwidth=%d];' % (a, b, loss, width))
    out.append("}")
    return "\n".join(out)


def main():
    args = [a for a in sys.argv[1:] if a != "--dot"]
    if not args:
        print(__doc__)
        return 1
    text = sys.stdin.read() if args[0] == "-" else open(args[0], encoding="utf-8",
                                                        errors="replace").read()
    parsed = parse(text)
    if parsed is None:
        print("Nie znalazlem zadnego zrzutu mapy. Wpisz MAP na konsoli wezla.")
        return 1
    collector, uptime, own, edges, routes = parsed
    if "--dot" in sys.argv[1:]:
        print(as_dot(collector, edges))
    else:
        print(as_text(collector, uptime, own, edges, routes))
    return 0


if __name__ == "__main__":
    sys.exit(main())

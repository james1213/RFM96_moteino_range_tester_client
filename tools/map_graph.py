# -*- coding: utf-8 -*-
"""Zamienia zrzuty mapy sieci ("MAP ..." na serialu) w opis tekstowy albo rysunek.

    python map_graph.py log.txt                          podsumowanie tekstowe
    python map_graph.py log.txt --svg mapa.svg           rysunek do otwarcia w przegladarce
    python map_graph.py log.txt --route 2:3              slad trasy z wezla 2 do 3
    python map_graph.py log.txt --svg mapa.svg --route 2:3
    python map_graph.py log.txt --dot                    Graphviz, jesli masz go zainstalowanego
    python map_graph.py mapa.log --follow --route 1:3    rysunek odswiezany na zywo
    python map_graph.py mapa.log --follow --html C:/mapy/siec.html
    python map_graph.py mapa.log --svg D:/dane/siec.svg --html C:/mapy/siec.html
    python map_graph.py mapa.log --max-age 300           szersze okno swiezosci
    type COM.log | python map_graph.py - --svg mapa.svg

SWIEZOSC DANYCH. Log rosnie godzinami, a mapa ma pokazywac stan TERAZ. Kazdy
zrzut i kazda krawedz dostaje wiec znacznik czasu z logu, a do rysunku trafia
tylko to, co jest nie starsze niz --max-age (domyslnie 90 s, czyli kilka cykli
odpytywania). Bez tego wystarczylo odlaczyc wezel, zeby jego lacza wisialy na
rysunku w nieskonczonosc - obraz byl suma calej historii, nie migawka.
Wezel, ktory znika z eteru, ale wciaz wystepuje w swiezych tablicach routingu
z kosztem 255, jest rysowany jako NIEOSIAGALNY: szare kolko bez lacz.
--max-age 0 wylacza filtr, gdy grzebiesz w starym logu.

Obok rysunku powstaje mala strona HTML, ktora sama przeladowuje obrazek co dwie
sekundy. Otwierasz ja raz w przegladarce i zostawiasz - mapa i trasa aktualizuja
sie w miare, jak wezly odpowiadaja na kolejne komendy MAP.

Gdzie trafiaja pliki:
    tylko --svg        strona obok rysunku, z ta sama nazwa i rozszerzeniem .html
    tylko --html       rysunek obok strony, z ta sama nazwa i rozszerzeniem .svg
    oba                kazdy tam, gdzie wskazano; strona sama trafi do rysunku
    zadne (--follow)   mapa.svg i mapa.html w biezacym katalogu
Brakujace katalogi sa zakladane. Strona jest zapisywana na nowo przy kazdym
uruchomieniu, wiec zawsze wskazuje aktualny rysunek.
Log karmi sie skryptem map_poll.ps1 albo przekierowana konsola programatora.

Rysunek jest generowany wprost do SVG - bez Graphviza i bez zadnej biblioteki
spoza standardowego Pythona, bo na tej maszynie nie ma ani jednego, ani drugiego.
Plik SVG otwiera sie dwuklikiem w przegladarce i skaluje bez utraty jakosci.

Format zrzutu (wezel wypisuje go po komendzie MAP):

    MAP BEGIN <wezel> <czas pracy s>
    MAP N <sasiad> <tlumienie dB>            lacze zmierzone przez ten wezel
    MAP E <a> <b> <tlumienie dB> <wiek s>    lacze uslyszane od kogos innego
    MAP R <cel> <przez> <koszt>              PIERWSZY SKOK trasy do celu
    MAP END
    MAP RESP <wezel> <liczba sasiadow>       naglowek odpowiedzi na MAP <id>

CALA TRASA Z JEDNEGO WEZLA. Routing jest skok po skoku, wiec wezel zna tylko
NASTEPNY SKOK do celu. Komenda MAP * kaze podlaczonemu wezlowi odpytac po kolei
wszystkie znane mu wezly o ich tablice tras; kazda odpowiedz wypisuje sie jako
osobny zrzut podpisany numerem swojego wezla. Skrypt sklada z tych tablic
lancuch i rysuje cala droge, choc do PC podlaczony jest tylko jeden wezel.
Czego brakuje, o tym powie wprost - poda numer wezla bez zrzutu.
"""
import argparse
import math
import os
import pathlib
import re
import sys
import time
import urllib.parse

# Linie z rejestratora bywaja poprzedzone znacznikiem czasu i prefiksem portu,
# np. "12:31:02 [COM] | MAP E 1 3 108 4" - bierzemy wszystko od slowa MAP.
LINE = re.compile(r"MAP\s+(BEGIN|END|RESP|[NER])\b(.*)")
STAMP = re.compile(r"^(\d\d):(\d\d):(\d\d)\b")

DAY = 24 * 3600
UNREACHABLE = 255

# Progi tlumienia zgodne z linkCost() w MeshRouter: do 70 dB skok kosztuje
# minimum, powyzej rosnie o 1 na kazde 8 dB.
QUALITY = ((70, "bardzo dobre", "#2e7d32"),
           (90, "dobre", "#7cb342"),
           (110, "slabe", "#f9a825"),
           (999, "na granicy", "#c62828"))


def quality(loss_db):
    for limit, name, colour in QUALITY:
        if loss_db <= limit:
            return name, colour
    return QUALITY[-1][1], QUALITY[-1][2]


def stamp_seconds(line):
    """Sekunda doby ze znacznika czasu rejestratora albo None, gdy linia go nie ma."""
    m = STAMP.match(line)
    if not m:
        return None
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))


def age_between(reference, moment):
    """Wiek w sekundach wzgledem punktu odniesienia, odporny na polnoc.

    Log ma same godziny, bez daty, wiec roznice liczymy modulo doba. Zapis sprzed
    chwili, ale po drugiej stronie polnocy, wyjdzie wtedy poprawnie maly.
    """
    return (reference - moment) % DAY


class Snapshot(object):
    """Stan sieci widziany w chwili odniesienia, po odsianiu starych danych."""

    def __init__(self, dumps, edges, reference, timed, max_age):
        self.reference = reference          # sekunda doby najnowszej linii MAP
        self.timed = timed                  # czy log w ogole ma znaczniki czasu
        self.max_age = max_age
        self.dropped_dumps = {}
        self.dropped_edges = {}
        self.dumps = {}
        self.edges = {}

        for node, dump in dumps.items():
            age = self._age(dump["t"])
            if self._fresh(age):
                self.dumps[node] = dump
            else:
                self.dropped_dumps[node] = age

        for pair, edge in edges.items():
            age = self._age(edge["t"])
            if self._fresh(age):
                self.edges[pair] = edge
            else:
                self.dropped_edges[pair] = age

        # Wezel, o ktorym swieze tablice mowia "koszt 255", istnieje, ale jest
        # teraz nieosiagalny. Rysujemy go, bo to informacja - tyle ze bez lacz.
        self.unreachable = set()
        for dump in self.dumps.values():
            for dest, (_, cost) in dump["routes"].items():
                if cost >= UNREACHABLE:
                    self.unreachable.add(dest)
        self.unreachable -= set(self.dumps)
        for a, b in self.edges:
            self.unreachable.discard(a)
            self.unreachable.discard(b)

    def _age(self, moment):
        if moment is None or self.reference is None:
            return 0
        return age_between(self.reference, moment)

    def _fresh(self, age):
        return self.max_age <= 0 or not self.timed or age <= self.max_age

    def nodes(self):
        found = set(self.dumps) | self.unreachable
        for a, b in self.edges:
            found.add(a)
            found.add(b)
        for dump in self.dumps.values():
            found.update(dump["routes"])
        return sorted(found)

    def reference_label(self):
        if not self.timed or self.reference is None:
            return "log bez znacznikow czasu"
        return "%02d:%02d:%02d" % (self.reference // 3600, (self.reference // 60) % 60,
                                   self.reference % 60)

    def wall_clock_age(self):
        """Ile sekund temu powstala najnowsza linia, wedlug zegara komputera.

        None, gdy logu nie da sie odniesc do teraz (brak znacznikow albo zapis
        starszy niz pol doby, czyli plik z innego dnia).
        """
        if not self.timed or self.reference is None:
            return None
        now = time.localtime()
        age = age_between(now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec, self.reference)
        return age if age < DAY / 2 else None


def parse(text, max_age=90):
    """Buduje migawke sieci z tekstu logu.

    Zrzuty i krawedzie pamietaja czas, w ktorym je uslyszano, wiec starsze dane
    da sie pozniej odsiac. Kazdy kolejny zrzut tego samego wezla zastepuje
    poprzedni.
    """
    dumps = {}
    edges = {}
    current = None
    latest = None
    timed = False

    for raw in text.splitlines():
        m = LINE.search(raw)
        if not m:
            continue
        moment = stamp_seconds(raw)
        if moment is not None:
            timed = True
            latest = moment
        kind, rest = m.group(1), m.group(2).split()
        try:
            if kind == "BEGIN":
                node = int(rest[0])
                current = {"uptime": int(rest[1]), "neighbours": {}, "routes": {}, "t": moment}
                dumps[node] = current
            elif kind == "END":
                current = None
            elif kind == "N" and current is not None:
                current["neighbours"][int(rest[0])] = int(rest[1])
            elif kind == "R" and current is not None:
                current["routes"][int(rest[0])] = (int(rest[1]), int(rest[2]))
            elif kind == "E":
                a, b, loss = int(rest[0]), int(rest[1]), int(rest[2])
                reported = int(rest[3]) if len(rest) > 3 else 0
                edges[(min(a, b), max(a, b))] = {"loss": loss, "reported": reported, "t": moment}
        except (IndexError, ValueError):
            continue  # linia ucieta w polowie przez inny watek logu

    # Wlasne, zmierzone lacza sa dokladniejsze od zaslyszanych - wpisujemy je na
    # koncu, z czasem swojego zrzutu.
    for node, dump in dumps.items():
        for neighbour, loss in dump["neighbours"].items():
            edges[(min(node, neighbour), max(node, neighbour))] = {
                "loss": loss, "reported": 0, "t": dump["t"]}

    return Snapshot(dumps, edges, latest, timed, max_age)


def trace(dumps, src, dst):
    """Slad trasy przez kolejne tablice routingu. Zwraca (lista wezlow, powod przerwania)."""
    path = [src]
    node = src
    while node != dst:
        dump = dumps.get(node)
        if dump is None:
            return path, "brak swiezego zrzutu z wezla %d - wpisz na nim MAP i dolacz log" % node
        route = dump["routes"].get(dst)
        if route is None:
            return path, "wezel %d nie ma w tablicy trasy do %d" % (node, dst)
        via, cost = route
        if cost >= UNREACHABLE:
            return path, "wezel %d ma trase do %d oznaczona jako nieosiagalna" % (node, dst)
        if via in path:
            return path, "petla routingu: %d wskazuje z powrotem na %d" % (node, via)
        path.append(via)
        node = via
        if len(path) > 10:
            return path, "trasa dluzsza niz 10 skokow - przerywam"
    return path, None


def freshness_note(snap):
    """Jedno zdanie o tym, jak stare sa dane - albo None, gdy nie ma o czym mowic."""
    if not snap.timed:
        return "log bez znacznikow czasu - nie odsiewam starych danych"
    if snap.max_age <= 0:
        return "filtr swiezosci wylaczony (--max-age 0)"
    wall = snap.wall_clock_age()
    if wall is not None and wall > snap.max_age:
        return "brak swiezych danych od %d s - wezly moga byc odlaczone" % wall
    return None


def as_text(snap, route):
    collectors = sorted(snap.dumps)
    out = ["Dane z %s%s" % (snap.reference_label(),
                           "" if snap.wall_clock_age() is None
                           else ", %d s temu" % snap.wall_clock_age())]
    note = freshness_note(snap)
    if note:
        out.append(note)
    out.append("")
    out.append("Zrzuty z wezlow: " + (", ".join(str(c) for c in collectors) or "brak"))
    out.append("Wezly: " + (", ".join(str(n) for n in snap.nodes()) or "brak"))
    if snap.unreachable:
        out.append("Nieosiagalne wedlug swiezych tablic: "
                   + ", ".join(str(n) for n in sorted(snap.unreachable)))
    out.append("")
    out.append("Lacza (tlumienie sciezki):")
    if not snap.edges:
        out.append("  brak swiezych lacz")
    for (a, b), edge in sorted(snap.edges.items()):
        name, _ = quality(edge["loss"])
        stale = "" if edge["reported"] == 0 else ", zaslyszane sprzed %d s" % edge["reported"]
        out.append("  %d - %d   %3d dB  (%s%s)" % (a, b, edge["loss"], name, stale))
    if snap.dropped_edges:
        out.append("  pominieto jako przestarzale: "
                   + ", ".join("%d-%d (%d s)" % (a, b, age)
                               for (a, b), age in sorted(snap.dropped_edges.items())))
    for node in collectors:
        routes = snap.dumps[node]["routes"]
        if not routes:
            continue
        out.append("")
        out.append("Pierwszy skok z wezla %d:" % node)
        for dest in sorted(routes):
            via, cost = routes[dest]
            how = "bezposrednio" if via == dest else "przez %d" % via
            bad = "  NIEOSIAGALNY" if cost >= UNREACHABLE else ""
            out.append("  do %d: %s (koszt %d)%s" % (dest, how, cost, bad))
    if route:
        src, dst = route
        path, why = trace(snap.dumps, src, dst)
        out.append("")
        out.append("Trasa %d -> %d: %s" % (src, dst, " -> ".join(str(n) for n in path)))
        if why:
            out.append("  dalej nie wiadomo: " + why)
    return "\n".join(out)


def layout(nodes, width, height):
    """Wezly na okregu. Prosto, przewidywalnie i czytelnie do kilkunastu wezlow."""
    cx, cy = width / 2.0, height / 2.0
    radius = min(width, height) / 2.0 - 80
    positions = {}
    count = max(1, len(nodes))
    for i, node in enumerate(nodes):
        angle = -math.pi / 2 + 2 * math.pi * i / count
        positions[node] = (cx + radius * math.cos(angle), cy + radius * math.sin(angle))
    return positions


def as_svg(snap, route, width=760, height=620):
    nodes = snap.nodes()
    pos = layout(nodes, width, height)
    path, why = trace(snap.dumps, route[0], route[1]) if route else ([], None)

    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
           'viewBox="0 0 %d %d" font-family="Segoe UI, sans-serif">' % (width, height, width, height),
           '<rect width="100%" height="100%" fill="#fbfbfb"/>',
           '<defs><marker id="grot" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" '
           'markerHeight="6" orient="auto-start-reverse">'
           '<path d="M 0 0 L 10 5 L 0 10 z" fill="#1565c0"/></marker></defs>']

    wall = snap.wall_clock_age()
    title = "Mapa sieci   dane z %s" % snap.reference_label()
    if wall is not None:
        title += " (%d s temu)" % wall
    if route:
        title += "   trasa %d - %d: %s" % (route[0], route[1],
                                           " - ".join(str(n) for n in path))
    svg.append('<text x="20" y="30" font-size="16" fill="#222">%s</text>' % title)

    warn_y = 52
    note = freshness_note(snap)
    if note:
        svg.append('<text x="20" y="%d" font-size="12" fill="#c62828">%s</text>' % (warn_y, note))
        warn_y += 18
    if route and why:
        svg.append('<text x="20" y="%d" font-size="12" fill="#c62828">dalej nie wiadomo: %s</text>'
                   % (warn_y, why))

    # Najpierw wszystkie lacza, zeby trasa mogla sie na nich polozyc.
    for (a, b), edge in sorted(snap.edges.items()):
        if a not in pos or b not in pos:
            continue
        (x1, y1), (x2, y2) = pos[a], pos[b]
        _, colour = quality(edge["loss"])
        dash = ' stroke-dasharray="6 4"' if edge["reported"] > 30 else ''
        svg.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
                   'stroke-width="2"%s/>' % (x1, y1, x2, y2, colour, dash))
        svg.append('<text x="%.1f" y="%.1f" font-size="11" fill="#555" text-anchor="middle">'
                   '%d dB</text>' % ((x1 + x2) / 2, (y1 + y2) / 2 - 4, edge["loss"]))

    # Trasa: gruba podkladka pod laczami plus strzalka na kierunek kazdego skoku.
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        if a not in pos or b not in pos:
            continue
        (x1, y1), (x2, y2) = pos[a], pos[b]
        # Skracamy odcinek o promien kolka, zeby grot strzalki nie chowal sie pod wezlem.
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        svg.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#1565c0" '
                   'stroke-width="7" stroke-opacity="0.28"/>' % (x1, y1, x2, y2))
        svg.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#1565c0" '
                   'stroke-width="2" marker-end="url(#grot)"/>'
                   % (x1 + ux * 24, y1 + uy * 24, x2 - ux * 26, y2 - uy * 26))

    for node in nodes:
        x, y = pos[node]
        dashed = ""
        if route and node == route[0]:
            fill, stroke = "#1565c0", "#0d3c73"
        elif route and node == route[1]:
            fill, stroke = "#2e7d32", "#1b4d20"
        elif node in snap.unreachable:
            fill, stroke = "#f2f2f2", "#c62828"   # w swiezych tablicach: koszt 255
            dashed = ' stroke-dasharray="5 4"'
        elif node in snap.dumps:
            fill, stroke = "#ffffff", "#1565c0"   # wezel, z ktorego mamy swiezy zrzut
        else:
            fill, stroke = "#eeeeee", "#999999"   # znany tylko z cudzych beaconow
        text = "#ffffff" if fill.startswith("#1") or fill.startswith("#2") else "#222222"
        svg.append('<circle cx="%.1f" cy="%.1f" r="22" fill="%s" stroke="%s" '
                   'stroke-width="2"%s/>' % (x, y, fill, stroke, dashed))
        svg.append('<text x="%.1f" y="%.1f" font-size="15" text-anchor="middle" fill="%s">'
                   '%d</text>' % (x, y + 5, text, node))

    legend = ["biale kolko: swiezy zrzut z tego wezla",
              "szare: znane tylko z cudzych beaconow",
              "czerwona przerywana obwodka: nieosiagalny (koszt 255)",
              "okno swiezosci: %s" % ("wylaczone" if snap.max_age <= 0 else "%d s" % snap.max_age)]
    for i, item in enumerate(legend):
        svg.append('<text x="20" y="%d" font-size="11" fill="#666">%s</text>'
                   % (height - 61 + i * 15, item))
    svg.append('</svg>')
    return "\n".join(svg)


def as_dot(snap, route):
    path, _ = trace(snap.dumps, route[0], route[1]) if route else ([], None)
    hops = {(min(path[i], path[i + 1]), max(path[i], path[i + 1])) for i in range(len(path) - 1)}
    out = ["graph siec {", "  layout=neato;", "  node [shape=circle];"]
    for node in snap.nodes():
        if node in snap.unreachable:
            mark = ' [style="filled,dashed" fillcolor=white color=red]'
        elif node in snap.dumps:
            mark = ' [style=filled fillcolor=lightgrey]'
        else:
            mark = ''
        out.append("  %d%s;" % (node, mark))
    for (a, b), edge in sorted(snap.edges.items()):
        loss = edge["loss"]
        width = 6 if (a, b) in hops else max(1, 5 - (loss - 60) // 15)
        colour = ' color=blue' if (a, b) in hops else ''
        out.append('  %d -- %d [label="%d dB", penwidth=%d%s];' % (a, b, loss, width, colour))
    out.append("}")
    return "\n".join(out)


HTML_WRAPPER = """<!doctype html>
<meta charset="utf-8">
<title>Mapa sieci</title>
<style>
 body { margin: 0; background: #fbfbfb; font-family: "Segoe UI", sans-serif; }
 img { display: block; max-width: 100%; }
 #stan { position: fixed; right: 12px; top: 10px; font-size: 12px; color: #777; }
</style>
<div id="stan">czekam...</div>
<img id="mapa" src="__SVG__">
<script>
// Przegladarka trzyma SVG w cache, wiec doklejamy znacznik czasu do adresu.
setInterval(function () {
  var img = document.getElementById("mapa");
  img.src = "__SVG__?t=" + Date.now();
  document.getElementById("stan").textContent =
      "odswiezono " + new Date().toLocaleTimeString();
}, 2000);
</script>
"""


def resolve_outputs(svg, html):
    """Ustala pelne sciezki rysunku i strony. Podana jedna wyznacza druga obok siebie."""
    if svg and not html:
        stem = svg[:-4] if svg.lower().endswith(".svg") else svg
        html = stem + ".html"
    elif html and not svg:
        stem = html[:-5] if html.lower().endswith(".html") else html
        svg = stem + ".svg"
    elif not svg and not html:
        svg, html = "mapa.svg", "mapa.html"
    return os.path.abspath(svg), os.path.abspath(html)


def svg_reference(svg_path, html_path):
    """Adres rysunku widziany ze strony.

    Wzgledny, jesli tylko sie da - wtedy katalog z oboma plikami mozna przeniesc
    w calosci i dalej dziala. Na Windowsie nie ma sciezki wzglednej miedzy dwoma
    dyskami (np. C: i Z:), wiec wtedy pelny adres file://. Znaki spoza ASCII
    i spacje sa kodowane, bo trafiaja do atrybutu src.
    """
    try:
        rel = os.path.relpath(svg_path, os.path.dirname(html_path))
    except ValueError:
        return pathlib.Path(svg_path).as_uri()
    return urllib.parse.quote(rel.replace(os.sep, "/"))


def write_svg(snap, route, svg_path):
    os.makedirs(os.path.dirname(svg_path), exist_ok=True)
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(as_svg(snap, route))


def write_html(svg_path, html_path):
    """Strona zapisywana raz na uruchomienie - przerysowania dotycza tylko SVG,
    a otwarta karta sama przeladowuje obrazek."""
    os.makedirs(os.path.dirname(html_path), exist_ok=True)
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(HTML_WRAPPER.replace("__SVG__", svg_reference(svg_path, html_path)))
    print("Strona:  %s" % html_path)
    print("Rysunek: %s" % svg_path)
    print("Otworz strone w przegladarce i zostaw karte otwarta.")


def follow(log_path, svg_path, html_path, route, max_age,
           poll_seconds=1.0, keep_bytes=400000, redraw_seconds=5.0):
    """Czyta log w miare, jak rosnie, i przerysowuje mape.

    Czytamy binarnie i pilnujemy przesuniecia w bajtach, bo plik jest w tym czasie
    dopisywany przez inny proces. Gdy zmaleje (rejestrator wystartowal od nowa),
    zaczynamy od poczatku. Chwilowa odmowa dostepu do pliku nie konczy sledzenia -
    na Windowsie zdarza sie zawsze, gdy dwa procesy siegaja po ten sam plik naraz.

    Rysunek powstaje takze wtedy, gdy w logu NIC nie przybywa: inaczej po
    odlaczeniu wezla na ekranie zostawal ostatni obraz sprzed awarii, bez slowa
    o tym, ze dane sa stare.
    """
    text = ""
    offset = 0
    html_written = False
    last_redraw = 0.0
    print("Sledze %s - przerwij Ctrl+C" % log_path)
    while True:
        fresh_lines = False
        try:
            size = os.path.getsize(log_path)
        except OSError:
            time.sleep(poll_seconds)
            continue
        if size < offset:
            text, offset = "", 0
        if size > offset:
            try:
                with open(log_path, "rb") as f:
                    f.seek(offset)
                    chunk = f.read()
                    offset = f.tell()
            except OSError:
                # Windows potrafi odmowic dostepu na ulamek sekundy, gdy rejestrator
                # wlasnie dopisuje linie (albo gdy log lezy na dysku sieciowym).
                # To nie powod, zeby konczyc - probujemy przy nastepnym obiegu.
                time.sleep(poll_seconds)
                continue
            decoded = chunk.decode("utf-8", errors="replace")
            text += decoded
            if len(text) > keep_bytes:
                # Zostawiamy ogon na granicy linii - starsze zrzuty i tak sa
                # zastepowane przez nowsze.
                text = text[-keep_bytes:]
                text = text[text.find(chr(10)) + 1:]
            fresh_lines = "MAP" in decoded

        now = time.time()
        if text and (fresh_lines or now - last_redraw >= redraw_seconds):
            last_redraw = now
            snap = parse(text, max_age)
            if snap.dumps or snap.edges or snap.unreachable:
                write_svg(snap, route, svg_path)
                if not html_written:
                    write_html(svg_path, html_path)
                    html_written = True
                if fresh_lines:
                    line = "%s  wezly ze zrzutem: %s" % (
                        time.strftime("%H:%M:%S"),
                        ", ".join(str(n) for n in sorted(snap.dumps)) or "brak")
                    if snap.unreachable:
                        line += "   nieosiagalne: " + ", ".join(
                            str(n) for n in sorted(snap.unreachable))
                    if route:
                        path, why = trace(snap.dumps, route[0], route[1])
                        line += "   trasa %d->%d: %s" % (
                            route[0], route[1], " -> ".join(str(n) for n in path))
                        if why:
                            line += "  (" + why + ")"
                    print(line)
        time.sleep(poll_seconds)


def parse_route(value):
    try:
        src, dst = value.split(":")
        return int(src), int(dst)
    except ValueError:
        raise argparse.ArgumentTypeError("trase podaj jako <zrodlo>:<cel>, np. 2:3")


def main():
    ap = argparse.ArgumentParser(description="Rysuje mape sieci i slad trasy ze zrzutow MAP.")
    ap.add_argument("log", help="plik z logiem serialowym albo - dla standardowego wejscia")
    ap.add_argument("--svg", metavar="PLIK", help="zapisz rysunek SVG (bez zadnych zaleznosci)")
    ap.add_argument("--html", metavar="PLIK",
                    help="gdzie zapisac strone odswiezajaca rysunek; domyslnie obok SVG")
    ap.add_argument("--dot", action="store_true", help="wypisz graf w formacie Graphviz")
    ap.add_argument("--route", type=parse_route, metavar="A:B",
                    help="podswietl trase z wezla A do wezla B")
    ap.add_argument("--follow", action="store_true",
                    help="sledz rosnacy log i przerysowuj mape na biezaco")
    ap.add_argument("--max-age", type=int, default=90, metavar="SEK",
                    help="ile sekund dane pozostaja wazne; 0 wylacza filtr (domyslnie 90)")
    args = ap.parse_args()

    if args.follow:
        if args.log == "-":
            ap.error("--follow potrzebuje pliku, nie standardowego wejscia")
        svg_path, html_path = resolve_outputs(args.svg, args.html)
        try:
            follow(args.log, svg_path, html_path, args.route, args.max_age)
        except KeyboardInterrupt:
            print("\nkoniec")
        return 0

    text = sys.stdin.read() if args.log == "-" else open(args.log, encoding="utf-8",
                                                         errors="replace").read()
    snap = parse(text, args.max_age)
    if not snap.dumps and not snap.edges:
        print("Nie znalazlem swiezego zrzutu mapy. Wpisz MAP na konsoli wezla"
              " albo poszerz okno przelacznikiem --max-age.")
        return 1

    drawing = bool(args.svg or args.html)
    if drawing:
        svg_path, html_path = resolve_outputs(args.svg, args.html)
        write_svg(snap, args.route, svg_path)
        write_html(svg_path, html_path)
    if args.dot:
        print(as_dot(snap, args.route))
    if not drawing and not args.dot:
        print(as_text(snap, args.route))
    return 0


if __name__ == "__main__":
    sys.exit(main())

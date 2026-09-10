//
// Mesh: automatyczny routing po jakosci lacza (DSDV w wersji na 2 KB RAM).
//
// Jak mierzymy jakosc lacza
// -------------------------
// Kazdy beacon niesie moc, z jaka zostal nadany. Odbiorca zna wiec TLUMIENIE
// SCIEZKI: tlumienie = moc nadania - RSSI odbioru. To miara lacza, a nie ustawien
// nadajnika - dwa wezly blisko siebie maja male tlumienie niezaleznie od tego, czy
// akurat nadaja na 2 czy 20 dBm. Surowe RSSI by sie do tego nie nadawalo: spadek
// mocy nadawania (APC) wygladalby jak pogorszenie lacza.
//
// Jak wybieramy trase
// -------------------
// Koszt skoku = MESH_LINK_COST_BASE + 1 za kazde 8 dB tlumienia powyzej progu.
// Skladnik bazowy premiuje trasy o mniejszej liczbie skokow (kazdy skok to czas
// w eterze i kolejna szansa na kolizje). Trasa wygrywa, gdy ma mniejsza sume
// kosztow; przy remisie zostaje ta obecna (histereza MESH_ROUTE_SWITCH_MARGIN),
// bo wezly sa w ruchu i RSSI faluje o kilka dB.
//
#ifndef RFM96_MESH_ROUTER_H
#define RFM96_MESH_ROUTER_H

#pragma once

#include <Arduino.h>
#include <radiomanager/RadioManager.h>

#define MESH_BEACON_INTERVAL_MS   3000  // + jitter 0-511 ms, zeby beacony sie nie zderzaly
#define MESH_NEIGHBOR_TIMEOUT_MS  12000 // prawdziwa CISZA (zadnych ramek) = sasiad znikl;
                                        // same zgubione beacony tras nie usmiercaja
#define MESH_MAX_NEIGHBORS        4
#define MESH_MAX_ROUTES           6
#define MESH_DEDUP_SIZE           16 // musi pokryc horyzont retransmisji (~3 s watchdoga ACK)

#define MESH_MAX_TTL              4     // max skokow; dobija ramki, ktore ucieka dedupowi
#define MESH_HOP_RETRIES          2     // ponowienia jednego skoku (po ACK-timeoucie radia)
#define MESH_METRIC_INFINITY      255
#define MESH_ROUTE_SWITCH_MARGIN  2     // histereza: nowa trasa musi byc lepsza o tyle
#define MESH_LINK_GOOD_PATHLOSS   70    // dB; do tego tlumienia lacze kosztuje bazowe 4
#define MESH_LINK_COST_BASE       4     // koszt idealnego skoku (premiuje mniej skokow)

// ==================== BINARNY FORMAT WIADOMOSCI MESH ====================
// Tresc ramki radiowej typu RADIO_TYPE_MESH zaczyna sie bajtem rodzaju.
//
//   BEACON: [1][moc nadania int8][seq][liczba tras] potem 3 B na trase:
//           [cel][koszt][seq celu]
//   DANE:   [2][zrodlo][cel koncowy][TTL][id strumienia] potem tresc aplikacji
//
// Tekstowy odpowiednik ("B?10?42?2:8:13,3:6:44") kosztowal 8-12 znakow na trase
// i wymagal parsowania liczb przy kazdym odbiorze. Binarnie trasa to 3 bajty,
// a odczyt to zwykle indeksowanie tablicy.
#define MESH_MSG_BEACON       1
#define MESH_MSG_DATA         2
#define MESH_MSG_TOPO_REQ     3 // "opowiedz o sobie: sasiedzi i tablica tras"
// Odpowiedz ma DWIE sekcje, jedna po drugiej:
//   [liczba sasiadow] potem 2 B na sasiada: [id][tlumienie dB]
//   [liczba tras]     potem 3 B na trase:   [cel][nastepny skok][koszt]
// Sekcja tras jest tym, co pozwala odczytac CALA trase z jednego wezla przy PC:
// routing jest skok po skoku, wiec zeby przejsc droge do konca, trzeba znac
// nastepny skok kazdego posrednika. Najwiekszy rozmiar to 1+8+1+18 = 28 bajtow.
#define MESH_MSG_TOPO_RESP    4
#define MESH_ROUTE_ENTRY      3
#define MESH_TOPO_REPORT_MAX  (1 + MESH_NEIGHBOR_ENTRY * MESH_MAX_NEIGHBORS \
                               + 1 + MESH_ROUTE_ENTRY * MESH_MAX_ROUTES)
// Odstep miedzy kolejnymi pytaniami przy odpytywaniu wszystkich wezlow po kolei.
// Jedno pytanie naraz, bo warstwa radiowa ma jeden slot transakcji.
#define MESH_WALK_GAP_MS      1500
#define MESH_BEACON_HEADER    4
#define MESH_BEACON_ROUTE_LEN 3
#define MESH_DATA_HEADER      5
#define MESH_NEIGHBOR_ENTRY   2

// ==================== MAPA SIECI ====================
// Beacon niesie teraz takze LISTE SASIADOW nadawcy (2 bajty na sasiada: id i
// tlumienie sciezki). Kazdy wezel sklada z tego obraz sieci na DWA SKOKI: wlasnych
// sasiadow zna z pomiaru, a ich sasiadow z ich beaconow. W tekstowym protokole
// takie pole kosztowaloby ~8 znakow na sasiada i nie zmiescilo by sie rozsadnie
// w ramce - binarny naglowek zrobil na nie miejsce.
//
// Dalsze wezly (3 skoki i wiecej) odpytuje sie jawnie: MESH_MSG_TOPO_REQ leci
// trasa z tablicy routingu, a odpowiedz wraca ta sama droga i laduje w tablicy
// krawedzi. Wezel przy PC jest wiec kolektorem: "MAP" zrzuca to, co wie, a
// "MAP <id>" dopytuje wskazany wezel.
#define MESH_MAX_EDGES        10
#define MESH_EDGE_TIMEOUT_MS  60000 // krawedz nieodswiezona przez minute znika z mapy

// Tresc dostarczona przez mesh: wskaznik w bufor odbiorczy radia (zakonczony
// zerem, wiec nadaje sie wprost na C-string), dlugosc i WEZEL ZRODLOWY - nie
// nadawca ostatniego skoku.
typedef void (*MeshDataCallback)(const char *payload, uint8_t len, uint8_t origin);


class MeshRouter {
public:
    MeshRouter(RadioManager *manager);

    void loop();                                   // beacony + starzenie tablic
    bool send(uint8_t finalDest, const uint8_t *payload, uint8_t len,
              void (*okCallback)() = nullptr,
              RadioFailCallback failCallback = nullptr);
    void onDataReceived(MeshDataCallback callback);
    void radioMeshDataReceived(uint8_t *payload, uint8_t len, uint8_t radioSender); // wpiecie z main.cpp
    void noteFrameFrom(uint8_t senderId);          // dowod zycia sasiada z KAZDEJ ramki
    void setFrozen(bool frozen);                   // OTA: bez beaconow i forwardingu
    uint8_t getNextHop(uint8_t dest);              // 0 = brak trasy
    uint8_t getRouteMetric(uint8_t dest);          // MESH_METRIC_INFINITY = brak
    void printState();                             // jedna linia: sasiedzi, trasy, flagi (czarna skrzynka)
    bool requestTopology(uint8_t dest);            // zapytaj wezel o sasiadow i trasy
    void requestTopologyAll();                     // odpytaj po kolei wszystkie znane wezly
    void printMap();                               // zrzut calej znanej mapy (format dla PC)

private:
    struct Neighbor {
        uint8_t id = 0;                 // 0 = wolny wpis
        uint16_t pathLossDb = 0;        // EMA tlumienia lacza (dB)
        unsigned long lastHeardMillis = 0;
    };
    struct Route {
        uint8_t dest = 0;               // 0 = wolny wpis
        uint8_t nextHop = 0;
        uint8_t metric = MESH_METRIC_INFINITY;
        uint8_t seq = 0;                // numer sekwencyjny celu (DSDV)
    };
    // Krawedz grafu sieci, ktorej NIE jestesmy koncem - z listy sasiadow w cudzym
    // beaconie albo z odpowiedzi na zapytanie o topologie. Wlasne lacza sa
    // dokladniejsze i mieszkaja w tablicy sasiadow.
    struct MapEdge {
        uint8_t a = 0;                  // 0 = wolny wpis; zawsze a < b, zeby
        uint8_t b = 0;                  // ta sama krawedz nie weszla dwa razy
        uint8_t pathLossDb = 0;
        unsigned long heardMillis = 0;
    };

    RadioManager *manager;
    MeshDataCallback dataReceivedCallback = nullptr;

    Neighbor neighbors[MESH_MAX_NEIGHBORS];
    Route routes[MESH_MAX_ROUTES];
    uint8_t dedupOrigin[MESH_DEDUP_SIZE] = {0};
    uint8_t dedupId[MESH_DEDUP_SIZE] = {0};
    uint8_t dedupNext = 0;

    uint8_t ownSeq = 0;                 // rosnie z kazdym NADANYM beaconem
    uint8_t nextFlowId = 1;             // id wlasnych ramek danych (dedup u innych)
    unsigned long lastBeaconMillis = 0;
    uint16_t beaconDueInMs = 800;       // pierwszy beacon szybko po starcie
    bool frozen = false;

    // Jeden skok "w locie" naraz: kontekst do ponowien po ACK-timeoucie.
    uint8_t hopRetriesLeft = 0;
    uint8_t hopDest = 0;
    void (*appOkCallback)() = nullptr;              // callback aplikacji dla wlasnej wysylki
    RadioFailCallback appFailCallback = nullptr;
    // Timeout ACK trafil w zajete radio: ponowienie odlozone, obslugiwane w loop().
    bool hopRetryPending = false;
    unsigned long hopRetryAtMillis = 0;
    unsigned long hopRetryDeadlineMillis = 0;
    // Jedno gniazdo odlozonego forwardu: relay dostal ramke w trakcie wlasnej
    // transakcji ACK, a poprzedni skok juz ja potwierdzil - nikt jej nie ponowi.
    // Bufor jest staly: kopia i tak musi powstac (tresc lezy w buforze odbiorczym,
    // ktory nadpisze nastepna ramka), a staly bufor nie moze zawiesc ani
    // pofragmentowac sterty.
    MapEdge edges[MESH_MAX_EDGES];
    // Odpowiedz na zapytanie o topologie jest odkladana do petli glownej: przychodzi
    // w srodku obslugi odbioru, gdy slot transakcji ACK bywa zajety.
    uint8_t topoRespPendingTo = 0;
    // Odpytywanie po kolei: indeks w tablicy tras i chwila nastepnego pytania.
    bool walkActive = false;
    uint8_t walkIndex = 0;
    unsigned long walkNextAtMillis = 0;
    uint8_t pendingForward[RADIO_PAYLOAD_CAPACITY];
    uint8_t pendingForwardLen = 0;
    uint8_t pendingForwardHop = 0;
    unsigned long pendingForwardDeadline = 0;

    static MeshRouter *instance;        // trampolina dla callbackow bez kontekstu
    static void sHopAckOk();
    static void sHopAckFail(const uint8_t *payload, uint8_t len);
    void hopAckFail();
    void giveUpHop();

    bool sendBeacon();
    static uint8_t composeDataFrame(uint8_t *out, uint8_t msgType, uint8_t origin, uint8_t finalDest,
                                    uint8_t ttl, uint8_t flowId, const uint8_t *payload,
                                    uint8_t payloadLen);
    bool sendTyped(uint8_t msgType, uint8_t finalDest, const uint8_t *payload, uint8_t len,
                   void (*okCallback)(), RadioFailCallback failCallback);
    void handleBeacon(const uint8_t *body, uint8_t len, uint8_t radioSender);
    void handleData(uint8_t msgType, uint8_t *body, uint8_t len, uint8_t radioSender);
    bool forwardData(uint8_t msgType, uint8_t origin, uint8_t finalDest, uint8_t ttl,
                     uint8_t flowId, const uint8_t *payload, uint8_t payloadLen);
    uint8_t buildNeighborList(uint8_t *out); // [liczba][id][tlumienie]... - zwraca dlugosc
    uint8_t buildTopologyReport(uint8_t *out); // sasiedzi + tablica tras
    void topologyWalkLoop();                 // kolejne pytania przy odpytywaniu wszystkich
    void addEdge(uint8_t a, uint8_t b, uint8_t pathLossDb);
    void ageEdges();
    Neighbor *findNeighbor(uint8_t id, bool create);
    Route *findRoute(uint8_t dest, bool create);
    uint8_t linkCost(const Neighbor &n);
    void invalidateRoutesVia(uint8_t neighborId);
    void ageTables();
    bool isDuplicate(uint8_t origin, uint8_t flowId);
};

#endif //RFM96_MESH_ROUTER_H

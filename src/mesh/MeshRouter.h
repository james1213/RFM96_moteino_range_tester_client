//
// MeshRouter - prosta siec kratowa (mesh) nad RadioManagerem.
//
// ZASADA DZIALANIA
//   Kazdy wezel co MESH_BEACON_INTERVAL_MS rozglasza beacon (broadcast, bez ACK)
//   ze swoja tablica tras: "<MSH>B?<mocTx>?<seq>?<cel>:<koszt>:<seq>,...".
//   Odbiorca beaconu:
//     - mierzy TLUMIENIE LACZA do nadawcy: pathLoss = mocTx(z beaconu) - RSSI.
//       To miara niezalezna od mocy nadawania (ktora u nas zmienia APC w locie),
//       wygladzana EMA, bo RSSI skacze o +-kilka dB miedzy ramkami.
//     - aktualizuje trasy w stylu DSDV (wektor odleglosci z numerami sekwencyjnymi):
//       koszt trasy = koszt lacza do sasiada + koszt ogloszony przez sasiada.
//       Nowszy numer sekwencyjny celu zawsze wygrywa (to lamie petle); przy tym
//       samym numerze trasa zmienia sie tylko, gdy jest lepsza o histereze -
//       inaczej trasy trzepotalyby przy kazdym wahnieciu RSSI w ruchu.
//   Dane: "<MSH>D?<zrodlo>?<cel>?<ttl>?<id>?<tresc>" - unicast skok po skoku,
//   kazdy skok z ACK radiowym i ponowieniami; po wyczerpaniu ponowien trasy przez
//   tego sasiada sa uniewazniane od razu (wezly sa w ruchu - nie czekamy na beacon).
//   Duplikaty wykrywa pierscien (zrodlo, id) - ramka krazaca wraca i ginie, TTL
//   dobija reszte.
//
// BEACONY A MOC NADAWANIA
//   Beacony ida ZAWSZE na suficie mocy (apcMaxDbm), niezaleznie od biezacej
//   regulacji: APC potrafi zejsc do 2 dBm dla bliskiego sasiada i odlegle wezly
//   przestalyby nas slyszec - topologia zwinelaby sie do jednego lacza. Krotki
//   beacon co kilka sekund na pelnej mocy to pomijalny koszt energii, a moc
//   wpisana w beacon i tak czyni pomiar tlumienia poprawnym przy KAZDEJ mocy.
//
// CZEGO TA WERSJA NIE ROBI (swiadomie):
//   - ACK tylko skok po skoku; potwierdzenie "OK" u nadawcy oznacza dotarcie do
//     PIERWSZEGO posrednika, nie do celu.
//   - OTA nie przechodzi przez mesh - zostaje bezposrednie (klient -> cel).
//   - Metryka nie niesie liczby skokow, wiec trasy dluzsze niz MESH_MAX_TTL moga
//     zostac wyuczone - dane na nich zgina na TTL mimo poprawnych ACK skokowych.
//     Przy skali <= 4 wezlow bez znaczenia.
//

#ifndef RFM96_MESH_ROUTER_H
#define RFM96_MESH_ROUTER_H

#include <Arduino.h>
#include <radiomanager/RadioManager.h>

#define MESH_BEACON_INTERVAL_MS   3000  // + jitter 0-511 ms, zeby beacony sie nie zderzaly
#define MESH_NEIGHBOR_TIMEOUT_MS  12000 // prawdziwa CISZA (zadnych ramek) = sasiad znikl;
                                        // beacony gina w kolizjach, wiec zyciem sasiada
                                        // jest KAZDA odebrana od niego ramka, nie sam beacon
#define MESH_MAX_NEIGHBORS        4
#define MESH_MAX_ROUTES           6
#define MESH_DEDUP_SIZE           16 // musi pokryc horyzont retransmisji (~3 s watchdoga ACK)
                                     // przy kilku wpisach/s na ruchliwym relayu
#define MESH_MAX_TTL              4     // max skokow; dobija ramki, ktore ucieka dedupowi
#define MESH_HOP_RETRIES          2     // ponowienia jednego skoku (po ACK-timeoucie radia)
#define MESH_METRIC_INFINITY      255
#define MESH_ROUTE_SWITCH_MARGIN  2     // histereza: nowa trasa musi byc lepsza o tyle
#define MESH_LINK_GOOD_PATHLOSS   70    // dB; do tego tlumienia lacze kosztuje bazowe 4
#define MESH_LINK_COST_BASE       4     // koszt idealnego skoku (premiuje mniej skokow)

class MeshRouter {
public:
    MeshRouter(RadioManager *manager);

    void loop();                                   // beacony + starzenie tablic
    bool send(uint8_t finalDest, const String &payload,
              void (*okCallback)() = nullptr,
              void (*failCallback)(String &payload) = nullptr);
    void onDataReceived(void (*callback)(String &payload, uint8_t origin));
    void radioMeshDataReceived(String &str, uint8_t radioSender); // wpiecie z main.cpp
    void noteFrameFrom(uint8_t senderId);          // dowod zycia sasiada z KAZDEJ ramki
    void setFrozen(bool frozen);                   // OTA: bez beaconow i forwardingu
    uint8_t getNextHop(uint8_t dest);              // 0 = brak trasy
    uint8_t getRouteMetric(uint8_t dest);          // MESH_METRIC_INFINITY = brak
    void printTopology();                          // zrzut sasiadow i tras na serial

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

    RadioManager *manager;
    void (*dataReceivedCallback)(String &payload, uint8_t origin) = nullptr;

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
    void (*appFailCallback)(String &payload) = nullptr;
    // Timeout ACK trafil w zajete radio: ponowienie odlozone, obslugiwane w loop().
    bool hopRetryPending = false;
    unsigned long hopRetryAtMillis = 0;
    unsigned long hopRetryDeadlineMillis = 0;
    // Jedno gniazdo odlozonego forwardu: relay dostal ramke w trakcie wlasnej
    // transakcji ACK, a poprzedni skok juz ja potwierdzil - nikt jej nie ponowi.
    String pendingForwardFrame;
    uint8_t pendingForwardHop = 0;
    unsigned long pendingForwardDeadline = 0;

    static MeshRouter *instance;        // trampolina dla callbackow bez kontekstu
    static void sHopAckOk();
    static void sHopAckFail(String &taggedPayload);
    void hopAckFail(String &taggedPayload);
    void giveUpHop();

    bool sendBeacon();
    void handleBeacon(const char *body, uint8_t radioSender);
    void handleData(String &str, const char *body, uint8_t radioSender);
    bool forwardData(uint8_t origin, uint8_t finalDest, uint8_t ttl, uint8_t flowId,
                     const char *payload);
    Neighbor *findNeighbor(uint8_t id, bool create);
    Route *findRoute(uint8_t dest, bool create);
    uint8_t linkCost(const Neighbor &n);
    void invalidateRoutesVia(uint8_t neighborId);
    void ageTables();
    bool isDuplicate(uint8_t origin, uint8_t flowId);
};

#endif //RFM96_MESH_ROUTER_H

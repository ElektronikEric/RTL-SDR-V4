# ADS-B Decoder – Projektübersicht

## Worum geht es?

Dieses Hobby-Projekt empfängt Funksignale, die Flugzeuge automatisch
aussenden, um ihre Position, Höhe und weitere Flugdaten bekanntzugeben.
Dieses Verfahren heißt **ADS-B** (Automatic Dependent Surveillance–
Broadcast) und wird von praktisch allen modernen Verkehrsflugzeugen
genutzt. Genau diese Signale werten auch bekannte Flugradar-Webseiten
wie Flightradar24 aus.

Mit einem günstigen USB-Funkempfänger (einem sogenannten **RTL-SDR-
Stick**) und einer kleinen Antenne lassen sich diese Signale selbst
empfangen und auswerten – ganz ohne teure Spezialhardware. Das ist der
Kern dieses Projekts: die Signale einzufangen, daraus die enthaltenen
Flugzeugdaten herauszulesen und diese am Ende übersichtlich auf einer
Karte darzustellen.

## Warum ein eigenes Programm, wenn es fertige Tools gibt?

Es gibt bereits ausgereifte, freie Programme für genau diesen Zweck,
allen voran **dump1090**. Dieses Projekt orientiert sich fachlich stark
daran, wird aber bewusst selbst entwickelt – als Lernprojekt, um zu
verstehen, wie Funksignale in verwertbare Daten übersetzt werden, und um
die Verarbeitung Schritt für Schritt selbst nachzuvollziehen und bei
Bedarf anzupassen.

## Die verwendete Hardware

- **RTL-SDR-Stick** (USB-Funkempfänger), eingestellt auf die feste
  Frequenz 1090 MHz, auf der ADS-B-Signale gesendet werden
- Eine einfache Antenne, idealerweise mit freier Sicht zum Himmel
- Ein Windows-Rechner, auf dem das Auswerteprogramm läuft

## Wie die Verarbeitung grob abläuft

Vom rohen Funksignal bis zur fertigen Information "Flugzeug X befindet
sich hier" durchläuft das Signal mehrere Verarbeitungsschritte:

```
Funksignal (Antenne)
      │
      ▼
1. Empfang durch den RTL-SDR-Stick
      │
      ▼
2. Umwandlung in ein Signalstärke-Muster
      │
      ▼
3. Suche nach dem charakteristischen Nachrichtenbeginn
      │
      ▼
4. Übersetzung des Signals in eine Bitfolge (die eigentliche Nachricht)
      │
      ▼
5. Prüfung, ob die Nachricht fehlerfrei übertragen wurde
      │
      ▼
6. Auswertung: welche Flugzeugdaten stecken in der Nachricht?  (in Arbeit)
      │
      ▼
7. Darstellung auf einer Karte im Browser                     (geplant)
```

Im Folgenden werden diese sieben Schritte etwas genauer erklärt.

### 1. Empfang

Der RTL-SDR-Stick "hört" kontinuierlich auf der Frequenz 1090 MHz und
liefert die empfangenen Funkwellen als fortlaufenden Strom von
Rohdaten an den PC. Diese Rohdaten enthalten zunächst noch keine
erkennbare Struktur – sie müssen erst aufbereitet werden.

### 2. Signalaufbereitung

Aus den Rohdaten wird berechnet, wie *stark* das Signal zu jedem
Zeitpunkt gerade ist. Man kann sich das wie eine Lautstärke-Anzeige
vorstellen: Immer wenn ein Flugzeug gerade sendet, schlägt die Anzeige
kurz nach oben aus. Diese Stärke-Information ist die Grundlage für alle
weiteren Schritte.

### 3. Erkennung des Nachrichtenbeginns ("Preamble")

Jede ADS-B-Nachricht beginnt mit einem immer gleichen, sehr kurzen
Muster aus Pulsen – vergleichbar mit einer Art "Vorspann", der
signalisiert: "Jetzt beginnt eine Nachricht." Das Programm durchsucht
das Signal fortlaufend nach genau diesem Muster, um den Startpunkt
jeder Nachricht zu finden.

### 4. Dekodierung der Nachricht

Ab dem gefundenen Startpunkt wird das Signal in seine einzelnen Bits
(Nullen und Einsen) übersetzt. Eine vollständige Nachricht besteht aus
112 solcher Bits. Am Ende dieses Schritts liegt die Nachricht als reine
Bitfolge vor – so, wie sie das Flugzeug ursprünglich gesendet hat.

### 5. Fehlerprüfung

Funkübertragungen sind störanfällig – durch Rauschen, Reflexionen oder
schwache Signale können einzelne Bits verfälscht werden. Deshalb enthält
jede Nachricht eine eingebaute Prüfsumme. Das Programm berechnet diese
Prüfsumme selbst und vergleicht sie mit der im Signal übertragenen. Nur
wenn beide übereinstimmen, gilt die Nachricht als vertrauenswürdig und
wird weiterverarbeitet – alles andere wird verworfen.

### 6. Auswertung der Flugzeugdaten *(in Arbeit)*

Eine gültige Nachricht enthält verschlüsselt verschiedene Informationen,
je nach Nachrichtentyp zum Beispiel:

- die eindeutige **Kennung** des Flugzeugs (vergleichbar einem
  Kfz-Kennzeichen) und sein **Rufzeichen** (Callsign)
- die aktuelle **Flughöhe**
- **Position** (geografische Breite/Länge) – hierfür müssen jeweils
  zwei zusammengehörige Nachrichten kombiniert werden
- **Geschwindigkeit** und **Kurs**

Dieser Auswertungsschritt ist die nächste große Ausbaustufe des
Projekts.

### 7. Kartenvisualisierung *(geplant)*

Sobald Position und weitere Daten eines Flugzeugs bekannt sind, sollen
sie live auf einer interaktiven Karte im Browser erscheinen – ähnlich
wie bei Flightradar24, nur mit den eigenen, selbst empfangenen Daten.
Jedes erfasste Flugzeug erscheint dabei als Symbol an seiner Position,
inklusive kurzer Zusatzinfos (Höhe, Geschwindigkeit, Kennung) beim
Anklicken. Flugzeuge, von denen länger keine neue Nachricht mehr
empfangen wurde, verschwinden wieder von der Karte.

## Aktueller Stand

Die Schritte 1 bis 5 – also Empfang, Signalaufbereitung, Erkennung,
Dekodierung und Fehlerprüfung – funktionieren bereits. Das Programm
kann somit zuverlässig feststellen: "Hier ist gerade eine echte,
fehlerfreie ADS-B-Nachricht von einem Flugzeug mit dieser Kennung
angekommen."

Die inhaltliche Auswertung dieser Nachrichten (Schritt 6) sowie die
Kartendarstellung (Schritt 7) sind die nächsten geplanten Schritte.

## Ausblick

Nach Fertigstellung von Auswertung und Karte sind als weitere
Ausbaustufen unter anderem denkbar: eine dauerhafte Speicherung aller
empfangenen Nachrichten zur späteren Auswertung, eine Übersichtstabelle
aller aktuell sichtbaren Flugzeuge sowie eine optionale Anbindung an
bestehende Flugverfolgungs-Tools.

# Haftungsausschluss / Disclaimer

## Allgemein

Dieses Repository stellt Software zur **passiven Analyse von Funksignalen**
(z. B. wM-Bus, ACARS, ADS-B) mittels RTL-SDR-Hardware bereit. Es dient
ausschließlich Lern-, Hobby- und Forschungszwecken.

- Die Software **sendet keine Signale** und greift nicht aktiv in
  Funkverkehr, Flugsicherung oder andere technische Systeme ein.
- Sie wird empfangsseitig genutzt und wertet lediglich öffentlich
  ausgestrahlte Signale aus.

## Keine Gewährleistung

Die Software wird **"wie besehen" ("as is")** bereitgestellt, ohne jegliche
ausdrückliche oder stillschweigende Gewährleistung, einschließlich, aber
nicht beschränkt auf Gewährleistungen der Marktgängigkeit, der Eignung für
einen bestimmten Zweck und der Nichtverletzung von Rechten Dritter.

Es wird keine Garantie für Vollständigkeit, Richtigkeit oder Aktualität der
dekodierten Daten übernommen. Insbesondere:

- Dekodierte Positions-, Höhen- oder Geschwindigkeitsangaben können
  **fehlerhaft, unvollständig oder veraltet** sein.
- Die Software ist **nicht für sicherheitskritische Anwendungen**
  (z. B. Flugnavigation, Kollisionsvermeidung, Notfallsysteme) geeignet
  oder zertifiziert.

## Haftungsbeschränkung

Soweit gesetzlich zulässig, haften die Autor:innen und Mitwirkenden dieses
Repositories in keinem Fall für direkte, indirekte, zufällige, besondere,
exemplarische oder Folgeschäden (einschließlich, aber nicht beschränkt auf
Beschaffung von Ersatzgütern oder -dienstleistungen, Nutzungsausfall,
Datenverlust oder entgangenen Gewinn), unabhängig von der Ursache und der
Haftungstheorie, sei es aus Vertrag, Gefährdungshaftung oder unerlaubter
Handlung (einschließlich Fahrlässigkeit), die in irgendeiner Weise aus der
Nutzung dieser Software entsteht, selbst wenn auf die Möglichkeit solcher
Schäden hingewiesen wurde.

## Rechtliche und regulatorische Hinweise

Die Nutzung von SDR-Hardware und der Empfang bestimmter Frequenzbänder
unterliegen nationalen Vorschriften (z. B. Bundesnetzagentur in
Deutschland, BAKOM in der Schweiz, RTR in Österreich). Nutzer:innen sind
selbst dafür verantwortlich, sich über die in ihrem Land geltenden
Vorschriften zu informieren und diese einzuhalten, insbesondere hinsichtlich:

- des Empfangs und der Verarbeitung von Funksignalen (z. B. Flugfunk,
  Betriebsfunk),
- des Datenschutzes bei der Verarbeitung personenbezogener oder
  personenbeziehbarer Daten (z. B. Flugzeugkennungen, Standortdaten),
- etwaiger Beschränkungen bei der Veröffentlichung oder Weitergabe
  empfangener Daten.

## Herkunft von Konzepten/Code

Teile der Signalverarbeitungsarchitektur in diesem Repository sind
konzeptionell an [dump1090](https://github.com/flightaware/dump1090)
(FlightAware, GPLv2) angelehnt. Dieser Hinweis dient der Transparenz und
ersetzt keine eigenständige Prüfung der Lizenzkompatibilität, falls
Codeteile direkt übernommen wurden.

---

*Dieser Disclaimer stellt keine Rechtsberatung dar. Bei Unsicherheiten zu
lizenzrechtlichen oder regulatorischen Fragen empfiehlt sich die
Konsultation einer fachkundigen Person.*

# pray - Projektskelett

Das Projektskelett enthält eine skelettartige Beispielimplementierung des pray
Interfaces. Um automatisches Testen und Benchmarken zu ermöglichen enthält das
Projekt ein Referenz-Makefile zum Übersetzen des Projekts.


Enthaltene Dateien:

* `include/pray.h`
* `lib/pray.cc`
* `ext/json.hpp`
* `ext/stb_image_write.h`
* `ext/tiny_obj_loader.h`
* `Makefile`

## `include/pray.h`
Referenz-Header-Datei als Beispiel, wie eine Vektor-Repräsentation aussehen kann.

## `src/pray.cc`
Enthält eine Beispielimplementierung, die die Verwendung der
externen Bibliotheken demonstriert.

## `ext/json.hpp`, `ext/stb_image_write.h`, `ext/tiny_obj_loader.h`
Externe Bibliotheken für die Ein- und Ausgabe.

## `Makefile`
Ein GNU-Makefile zum Kompilieren des Projekts, darf beliebig angepasst werden.
Das Projekt kann grundsätzlich in einer beliebigen Umgebung entwickelt werden.
Bei Änderungen am Makefile oder falls ein anderes Buildsystem zum Einsatz
kommt, muss das `.gitlab-ci.yml`-Skript entsprechend adaptiert werden.

[![✗](https://img.shields.io/badge/Release-v0.3.0-ffb600.svg?style=for-the-badge)](https://github.com/BautiEtcheverry/murmur)

[![✗](https://github.com/BautiEtcheverry/murmur/actions/workflows/pipeline.yaml/badge.svg?branch=development)](https://github.com/BautiEtcheverry/murmur/actions/workflows/pipeline.yaml)

# Murmur

A domain-specific language for specifying and simulating swarm-drone missions. Built with Flex and Bison on top of the course's [Flex-Bison-Compiler](https://github.com/Alpha-Theta-Gamma-Mu/Flex-Bison-Compiler) template.

## Team

| Name                 | File ID | Email                      |
| :------------------- | :-----: | :------------------------- |
| Benicio Leite        | 64181   | bleite@itba.edu.ar         |
| Juan Humphreys       | 65477   | jhumphreys@itba.edu.ar     |
| Bautista Etcheverry  | 65765   | betcheverry@itba.edu.ar    |

* [Language](#language)
* [Architecture](#architecture)
* [Requirements](#requirements)
* [Configuration](#configuration)
* [Commands](#commands)
* [Visualizer](#visualizer)
* [Tests](#tests)
* [CI/CD](#cicd)
* [Recommended Extensions](#recommended-extensions)

The final report (Stage III) is available at [`doc/Murmur - Informe Final.pdf`](doc/Murmur%20-%20Informe%20Final.pdf), and the original specification (Stage I) at [`doc/Murmur - Especificación.pdf`](doc/Murmur%20-%20Especificación.pdf). The specification is kept for history and is **superseded by the final report**: the syntax changed after the reviews (`drone` instead of `drone_type`, `circle at` instead of `circle center`, constraints inside the mission), as summarized in Annex A of the report.

## Language

A Murmur program declares drone types, fleets, zones, stations and formations, and a single mission that commands the fleets over a time window:

```text
drone Quad {
  endurance 35 [min];
  max_speed 15 [m/s];
  payload thermal_camera resolution 640 [px] fov 57 [deg];
}
fleet Alpha = 4 of Quad;
zone Campo  = polygon [(0,0), (800,0), (800,500), (0,500)];
zone Antena = circle at (600, 400) radius 60 [m];
station Base = point (10, 10) altitude 0 [m];

mission Relevamiento window 0 [s] .. 25 [min] {
  Alpha takeoff from Base at 0 [s];
  Alpha morph to line spacing 30 [m] over 20 [s];
  Alpha sweep Campo with lawnmower spacing 40 [m] altitude 60 [m];
  all return to Base before 1400 [s];

  constraints {
    min_separation 20 [m];
    battery_reserve 20 [%];
    exclude Antena;
    require coverage of Campo >= 90 [%];
  }
}
```

Semicolons are optional. Every magnitude carries an explicit unit (`[s]`, `[min]`, `[m]`, `[km]`, `[m/s]`, `[km/h]`, `[m/s^2]`, `[deg]`, `[rad]`, `[px]`, `[%]`), and units of the same dimension are interchangeable. See [`src/test/c/accept`](src/test/c/accept) for one example program per feature.

## Architecture

| Phase | Location | Responsibility |
| :---- | :------- | :------------- |
| Lexical analysis | `src/main/c/frontend/lexical-analysis` | Flex scanner: keywords, units, literals, identifiers, comments. |
| Syntactic analysis | `src/main/c/frontend/syntactic-analysis` | Bison grammar (one terminal per physical dimension, so a unit of the wrong dimension is a syntax error) and AST construction. |
| Semantic analysis | `src/main/c/backend/semantic-analysis` | Symbol tables (declaration before use, single namespace), value ranges, convex polygons, mandatory fields, formation cardinality, mission window, parallel conflicts, coverage pre-conditions. |
| Simulation | `src/main/c/backend/simulation` | Reconstructs every trajectory by discrete sampling, and verifies `max_speed`, `max_accel`, `min_separation`, excluded zones, `link_range`, energy (`endurance`, `battery_reserve`) and coverage. |
| Code generation | `src/main/c/backend/code-generation` | Emits the JSON flight log. |
| Runtime | `src/main/visualizer` | HTML + SVG + JS visualizer (charts drawn with Chart.js) that consumes the flight log. |

The compiler exits with status `0` if the program is accepted, and `1` otherwise. Every error is reported in the console, with the aircraft involved (e.g. `Scout#3`), the instant and the magnitude of the violation when possible.

## Requirements

* [Docker v28.3.2](https://www.docker.com/)

## Configuration

Set the following environment variables to control and configure the behaviour of the application:

| Name                  | Default | Description                                                                                                                                                           |
| :-------------------- | :-----: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ENVIRONMENT`         | `Local` | The active environment name. The available environments are: `Local`, `Development` and `Production`.                                                                 |
| `LOG_IGNORED_LEXEMES` | `true`  | When `true`, logs all of the ignored lexemes found with Flex at `DEBUGGING` level. To remove those logs from the console output set it to `false`.                    |
| `LOGGING_LEVEL`       | `ALL`   | The minimum level to log in the console output. From lower to higher, the available levels are: `ALL`, `DEBUGGING`, `INFORMATION`, `WARNING`, `ERROR` and `CRITICAL`. |

_Docker Compose_ can read the variables from an `.env` file too (see `compose.yaml` file).

## Commands

### Start

Rises an ephemeral container, ready to start development:

```bash
docker compose run --rm compiler
```

### Build

Builds or rebuilds the entire compiler:

```bash
src/main/bash/build.sh
```

### Run

Compiles a program:

```bash
src/main/bash/run.sh <program> [<output>]
```

where `<program>` is the path to the file that represents its entry-point, and `<output>` is the path of the JSON flight log to generate (by default, `out/<mission>.json`).

### Test

Executes every available unit-test under `src/test/c` folder:

```bash
src/main/bash/test.sh
```

### Stop

Logout, destroy the ephemeral containers and shutdowns the cluster:

```bash
exit
docker compose down
```

### Docker

| Command                                 | Description                                             |
| :-------------------------------------- | :------------------------------------------------------ |
| `docker builder prune --all`            | Removes all builds and complete build cache.            |
| `docker compose --progress=plain build` | Forces a build or rebuild of the images in the cluster. |
| `docker image prune`                    | Removes all of the dangling images from Docker.         |
| `docker network prune`                  | Removes unused networks from Docker.                    |
| `docker volume prune`                   | Removes unused volumes from Docker.                     |

## Visualizer

The visualizer ([`src/main/visualizer/index.html`](src/main/visualizer/index.html)) is a static page that consumes the JSON flight log generated by the compiler. It shows:

* The indicators of the mission (duration, aircraft, distance) and the margin of every verified constraint (minimum separation, battery reserve, coverage).
* A map with the zones (excluded zones are hatched), the stations and the aircraft. Each aircraft drags the trail of its last 45 seconds; choosing one draws its whole route, with arrows for the direction of travel.
* The plan of maneuvers: one row per maneuver, with who does what and when; the maneuver in progress is highlighted.
* Three charts synchronized with the map: altitude and battery of each aircraft, and the distance between the two closest aircraft, each against its declared limit. Hovering a chart moves the mission to that instant.
* A table with the state of every aircraft at the selected instant.

1. Open `src/main/visualizer/index.html` in a browser.
2. Choose (or drag and drop) a flight log, such as `out/Relevamiento.json`.

Alternatively, serve the repository with any static web server (e.g. `python3 -m http.server`) and open `src/main/visualizer/index.html?src=/out/Relevamiento.json`.

The supporting code (loading, sampling, map, playback and maneuver plan) lives in [`src/main/visualizer/shared`](src/main/visualizer/shared).

### External libraries

| Library | Version | License | Used by | How it is loaded |
| :------ | :-----: | :------ | :------ | :--------------- |
| [Chart.js](https://www.chartjs.org) | 4.4.1 | MIT | The charts of the visualizer | Vendored, unmodified, in [`src/main/visualizer/shared/vendor`](src/main/visualizer/shared/vendor), so the view works without an internet connection. |

The compiler itself has no external dependencies.

### Flight log format

```text
{
  "format": "murmur-flight-log", "version": 1,
  "mission": {"name", "window": [start, end], "end"},
  "constraints": {"minSeparation", "batteryReserve", "coverage": [{"zone", "required", "achieved"}]},
  "phases": [{"label", "maneuver", "start", "end"}],
  "droneTypes": [{"name", "endurance", "maxSpeed", "maxAccel", "linkRange", "payload": {"sensor", "resolution", "fov"}}],
  "fleets": [{"name", "aircraft": [id]}],
  "zones": [{"name", "kind", "center", "radius", "vertices": [[x, y]], "excluded", "swept", "coverage"}],
  "stations": [{"name", "position": [x, y], "altitude"}],
  "aircraft": [{"id", "type", "fleet", "deployed", "flightTime", "distance", "batteryAtEnd"}],
  "timeline": {"dt": 1, "frames": [{"t", "drones": [{"id", "x", "y", "z", "phase", "battery"}]}]}
}
```

Every magnitude is expressed in SI units (seconds, meters, meters per second, radians), and ratios in `[0, 1]`.

## Tests

* `src/test/c/accept`: 27 programs that must be accepted (one per feature of the language).
* `src/test/c/reject`: 31 programs that must be rejected (lexical, syntactic, semantic and mission-verification errors).

## CI/CD

To trigger an automatic integration on every push or PR (_Pull Request_), you must activate _GitHub Actions_ in the _Settings_ tab. Use the following configuration:

| Key                                                        | Value                                               |
| :--------------------------------------------------------- | :-------------------------------------------------- |
| `Actions permissions`                                      | `Allow all actions and reusable workflows`          |
| `Allow GitHub Actions to create and approve pull requests` | `false`                                             |
| `Artifact and log retention`                               | `30 days`                                           |
| `Fork pull request workflows from outside collaborators`   | `Require approval for all outside collaborators`    |
| `Workflow permissions`                                     | `Read repository contents and packages permissions` |

## Recommended Extensions

* [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
* [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
* [Yash](https://marketplace.visualstudio.com/items?itemName=daohong-emilio.yash)

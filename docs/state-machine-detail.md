# State machine — firmware view

Every state, guard, gait and timeout in `software/Hexapod/Hexapod.cpp`. For the same machine
in operating terms, see [the manual](MANUAL.md#the-five-things-it-does).

The Mermaid below is the source. [`state-machine-detail.svg`](state-machine-detail.svg) is the
same diagram exported for printing - **if you change one, change the other.**

```mermaid
flowchart LR
    Off["eOff"] -->|"isSwitchOn()<br>play(startupClip)<br>ParkGait 3000 ms"| Init["eInitializing"]
    Init -->|"!isSwitchOn()<br>setTorqueOff()<br>play(shutdownClip)"| Off
    Init -->|"isLinkHealthy()<br>re-park if !isAllTorqueOn()"| Ready["eReady"]
    Init -->|"60 s: setTorqueOff()"| Init
    Ready -->|"60 s: setTorqueOff()"| Ready
    Ready -->|"joyL &amp;&amp; canChangeGait()<br>StandUpGait 3000 ms"| Standing["eStanding"]
    Standing -->|"30 s idle<br>ParkGait 3000 ms"| Ready
    Standing -->|"any axis != 0 ||<br>joyR || switch2<br>walkingGaits[i]"| Walking["eWalking"]
    Walking -->|"15 s idle &amp;&amp; !switch2<br>StandUpGait 1000 ms"| Standing
    Walking -->|"joyR: ++gait index"| Walking
    Standing -->|"switch3<br>PosingGait"| Posing["ePosing"]
    Posing -->|"!switch3<br>StandUpGait 1000 ms"| Standing
    Standing -->|"switch4<br>LevelGait"| Leveled["eStandingLeveled"]
    Leveled -->|"!switch4<br>StandUpGait 1000 ms"| Standing
    Walking -.->|"switch3 &amp;&amp; !switch2<br>PosingGait"| Posing
    Walking -.->|"switch4 &amp;&amp; !switch2<br>LevelGait"| Leveled
    Leveled -.->|"switch3<br>PosingGait"| Posing

    N["<b>Common to the four active states</b><br>eStanding, eWalking, ePosing, eStandingLeveled<br>&nbsp;<br>joyL → ParkGait 3000 ms → eReady<br>!isLinkHealthy() || !isSwitchOn()<br>&nbsp;&nbsp;→ handleConnectionLoss(true) → eInitializing<br>&nbsp;&nbsp;the same from eReady, but with (false):<br>&nbsp;&nbsp;no gait is running there to interrupt"]
    Off ~~~ N

    linkStyle 14,15,16 stroke:#3A56A8,color:#3A56A8
    classDef s_off  fill:#E7E9EC,stroke:#3A4048,stroke-width:2px,color:#1A1E24
    classDef s_init fill:#FDE7D6,stroke:#E8590C,stroke-width:2px,color:#1A1E24
    classDef s_stnd fill:#FFF3BF,stroke:#B58B00,stroke-width:2px,color:#1A1E24
    classDef s_walk fill:#D9F2E0,stroke:#2B8A3E,stroke-width:2px,color:#1A1E24
    classDef s_mode fill:#E3E9FB,stroke:#3A56A8,stroke-width:2px,color:#1A1E24
    classDef s_note fill:#FFFDF3,stroke:#B58B00,stroke-width:1.5px,color:#1A1E24
    class Off s_off
    class Init,Ready s_init
    class Standing s_stnd
    class Walking s_walk
    class Posing,Leveled s_mode
    class N s_note
```

## Guards

`isLinkHealthy()` = `isPaired()` **and** `timeSinceLastControlData()` ≤ 250 ms. The two are
deliberately independent: the first is the transport layer's opinion, the second is the
control loop's own, and it catches a stalled WiFi task that would never clear the peer.

`canChangeGait()` gates every handler **except** `stepWalking()`. A state change from walking
therefore takes effect immediately while the gait switch is deferred to the end of the cycle;
the destination handler's own entry gate covers the gap.

## Constants

All in `Hexapod.cpp`.

| | |
|---|---|
| `cParkDurationMs` | 3000 |
| `cStandUpFromParkedMs` | 3000 |
| `cStandUpDeployedMs` | 1000 |
| `cIdleTimeout` | 15000 |
| `cWaitTimeUntilTorqueOff` | 60000 |
| `cControlDataTimeoutMs` | 250 |

## Regenerating the SVGs

Both SVGs are built with graphviz from the sources in [`diagrams/`](diagrams), then composed
so the note sits below the graph - `dot` lays nodes out by rank and cannot place a note
underneath a left-to-right graph on its own.

```
cd docs/diagrams
./build.sh          # needs graphviz and python3
```

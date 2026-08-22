# Plants

The `Plant` is **the truth**. A controller never sees inside it; it observes
only the noisy measurement returned by `step()`. All actuator saturation and
all transport delay live here, so every controller faces identical physical
limits.

| class | dynamics | what it is for |
|---|---|---|
| `Tank` | FOPDT, separate disturbance path | the workhorse of classical control |
| `IntegratingTank` | pure integrator + delay | surge tank — no self-regulation |
| `SeriesTanks` | N first-order lags | the half rule's honest test |
| `InverseResponseTank` | two opposing paths, RHP zero | drum level — shrink and swell |
| `CascadeProcess` | fast inner stage → slow outer stage | the case cascade exists for |

---

## Base class

::: process_control.plants.base.Plant

---

## Tank

::: process_control.plants.tank.Tank

---

## IntegratingTank

::: process_control.plants.integrating_tank.IntegratingTank

---

## SeriesTanks

::: process_control.plants.series_tanks.SeriesTanks

---

## InverseResponseTank

::: process_control.plants.inverse_response.InverseResponseTank

---

## CascadeProcess

::: process_control.plants.cascade_process.CascadeProcess

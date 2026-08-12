# Prompt for generating Stage 0 bench-sweep illustrations

Paste the block below into GPT-5 / Gemini (image generation, or "deep research"
for the source-hunting part). It is written to be self-contained — it carries
the vehicle dimensions and the measurement geometry, so the model does not have
to guess.

Generate the drawings **first**, look at them, and only then follow the written
procedure. If a drawing disagrees with the text, the drawing is probably wrong —
the text has been checked against the maths.

---

## Prompt — copy from here

> I am calibrating the steering of a 1/10-scale F1TENTH autonomous RC car and I
> need to measure the steering angle of each front wheel accurately, without a
> protractor. I have a written procedure but I cannot picture the rig. Please
> produce clear technical illustrations, then explain the procedure back to me
> in plain language.
>
> **The vehicle.** A 1/10-scale RC car chassis, roughly 550 mm long overall.
> Wheelbase (rear axle to front axle) 256 mm. Track (left wheel to right wheel)
> 203 mm. Wheels about 100 mm diameter and 40 mm wide. Rear wheels are fixed and
> cannot steer; front wheels steer together via a servo and a tie-rod linkage.
> There is no body shell — the chassis plate, motor, battery and electronics are
> exposed. The front of the car is currently propped up about 50 mm on a block
> placed under the chassis plate just behind the front wheels, so both front
> wheels hang free; the rear wheels rest on the floor.
>
> **The floor** is square tile. One straight grout line runs alongside the car's
> left-hand side, parallel-ish to the car, about 90 mm out from the wheels. That
> grout line is the measurement baseline.
>
> **The rig.** A 12-inch wooden ruler is taped flat against the outer vertical
> face of a wheel — the face pointing away from the car — lying horizontal at
> hub height and running front-to-back, so it acts as a long pointer showing
> which way that wheel is aimed. It is mounted deliberately off-centre: about
> 50 mm of ruler behind the wheel hub and about 250 mm sticking out ahead of the
> car into open air. Two small tape tabs mark two points on the ruler, 280 mm
> apart. Each front wheel gets its own ruler; the two rulers never touch each
> other or span between wheels.
>
> **The measurement.** A steel right-angle square (an L, one leg 12 inches, one
> leg 8 inches) is used to find how far each tape tab sits from the grout line.
> The long leg lies flat on the floor with its zero end on the grout line,
> pointing across toward the car, roughly perpendicular to the line. The short
> leg stands straight up in the air. The square is slid along the grout line
> until the vertical short leg just touches the ruler at one of the tape tabs.
> The distance is then read off the long leg where the vertical leg stands. The
> vertical leg is doing the job of a plumb line: the ruler is up in the air, the
> grout line is on the floor, and the square projects one down onto the other.
>
> Doing this at both tabs gives two distances, `front` and `rear`. The wheel's
> angle is `atan((rear − front) / 280 mm)`. If the front tab is closer to the
> grout line than the rear tab, the wheel is pointing left; if it is further
> away, the wheel is pointing right.
>
> **Please draw all of the following, as clean labelled technical line
> drawings — engineering-diagram style, not photorealistic. Black line art on
> white, thin leader lines, dimension arrows, minimal shading. Label every part
> in English.**
>
> 1. **Plan view (top-down, orthographic)** of the whole car with the grout line
>    along its left side, both front wheels turned to the left, and a ruler taped
>    to each front wheel with its tape tabs marked. Show the two perpendicular
>    measurement distances from the grout line to the front and rear tabs of the
>    left front wheel, as dimension arrows. This is the single most important
>    drawing.
> 2. **The same plan view three times side by side**, showing wheels turned
>    fully left, straight ahead, and fully right, so I can see how the two
>    distances change with steering angle.
> 3. **Front elevation (looking at the front of the car head-on, orthographic)**
>    showing the car propped on its block, the front wheels hanging clear of the
>    floor, the rulers taped to the outer faces of both front wheels, and the
>    square standing with its long leg on the floor and its short leg vertical,
>    touching a ruler. This is the view that shows the height relationship I am
>    confused about.
> 4. **Side elevation (looking at the car's left side, orthographic)** showing
>    the ruler taped across the front wheel, mounted off-centre with the long end
>    forward, at hub height, with the two tape tabs and the 280 mm dimension
>    between them.
> 5. **A detail view of just the square in use**, isometric, showing the long leg
>    flat on the floor with its zero on the grout line, the short leg vertical,
>    and the vertical leg touching a tape tab on the ruler. Include an arrow
>    showing the direction the square slides.
> 6. **An isometric overview** of the whole setup so I can see how everything
>    relates in three dimensions.
>
> Then, in text:
>
> - Explain the procedure back to me in simple steps, as if to someone who has
>   never done any mechanical measurement.
> - Explain why the reference has to come from the rear wheels rather than from
>   assuming the front wheels are straight.
> - Tell me the common mistakes that would make these readings wrong.
> - Search for and link real references — RC car wheel alignment guides, toe and
>   camber measurement tutorials, string-line alignment methods, DIY toe plates,
>   and any video showing measuring wheel angle against a reference line. RC
>   touring-car and 1/10-scale racing setup guides are the closest match to what
>   I am doing. Tell me which of those methods would be *better* than the one
>   described above, given that I have a tile floor, a steel square, two wooden
>   rulers, a tape measure and an iPhone.

## Also ask about the alternative that may replace all of this

> Separately: instead of the square-and-ruler method, would it be more accurate
> to photograph the car from directly overhead — phone held level, using the
> camera's built-in level indicator — with a ruler taped to each of the four
> wheels, and measure the wheel angles from the photograph in software? Angles
> lying in a plane parallel to the camera sensor are preserved under projection,
> so I believe this needs no scale reference and no calibration, only that the
> phone is level. Explain the accuracy limits of that approach, what would ruin
> it (lens distortion, tilt, parallax, rolling shutter), how much tilt is
> tolerable, and how best to set it up in a room with a normal ceiling height.

## Notes for whoever runs this

The overhead-photo alternative is the one the handoff recommends promoting to
primary — see `SYSID_HANDOFF.md`. Ask about it deliberately rather than treating
it as an afterthought; if the answers are as expected, most of the drawings
above become a fallback rather than the main path.

Numbers in the prompt are metric because they are the vehicle's real dimensions
(`config/vehicle/vesc.yaml`, `static_transformations.launch.py`). The operator's
instruments are imperial: 12 in rulers with 1/8 in graduations, and a 12 in ×
8 in square. `d = 280 mm` above is the 11 in span between tabs at the 0.5 in and
11.5 in marks. Keep both unit systems in front of the operator rather than
converting silently.

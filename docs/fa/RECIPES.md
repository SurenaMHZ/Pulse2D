<div dir="rtl">

# دستورالعمل‌ها

راه‌حل‌های آماده برای مسائلی که تقریباً در هر بازی پیش می‌آیند.

هر قطعه‌کد اینجا **روی موتور اجرا شده** و رفتار توصیف‌شده‌اش تأیید شده است —
اعدادی که در توضیحات می‌بینید اندازه‌گیری شده‌اند، نه حدسی. همهٔ این‌ها در
`test/recipes.test.mjs` به‌عنوان تست دائمی نگهداری می‌شوند.

[English version](../RECIPES.md)

**فهرست**

۱. [کنترلر کاراکتر](#۱-کنترلر-کاراکتر)
۲. [سکوی یک‌طرفه](#۲-سکوی-یکطرفه)
۳. [سکوی متحرک](#۳-سکوی-متحرک)
۴. [انفجار](#۴-انفجار)
۵. [نوار نقاله](#۵-نوار-نقاله)
۶. [یک خودروی ساده](#۶-یک-خودروی-ساده)
۷. [ناحیهٔ تریگر](#۷-ناحیهٔ-تریگر)
۸. [زمین از روی نقشهٔ ارتفاع](#۸-زمین-از-روی-نقشهٔ-ارتفاع)
۹. [مفصل شکستنی](#۹-مفصل-شکستنی)
۱۰. [کشیدن با ماوس](#۱۰-کشیدن-با-ماوس)
۱۱. [رندر نرم](#۱۱-رندر-نرم)
۱۲. [استخر اشیاء](#۱۲-استخر-اشیاء)

---

## ۱. کنترلر کاراکتر

یک کپسول با چرخش قفل‌شده، کنترل مستقیم سرعت برای حرکت، و یک پرتو برای بررسی
تماس با زمین.

</div>

```ts
const player = world.createBody({
  type: BodyType.Dynamic,
  position: { x: 0, y: 3 },
  fixedRotation: true,          // هرگز نمی‌افتد
});
player.addFixture({
  shape: Capsule.vertical(1.8, 0.4),   // قد ۱٫۸ متر
  density: 1,
  friction: 0.2,                // کم: سرعت را مستقیم کنترل می‌کنیم
});

const HALF_HEIGHT = 0.9;
const SKIN = 0.08;

function isGrounded() {
  const p = player.getPosition();
  const hit = world.rayCastClosest(
    Scalar.toFloat(p.x), Scalar.toFloat(p.y),
    Scalar.toFloat(p.x), Scalar.toFloat(p.y) - HALF_HEIGHT - SKIN,
    (fixture) => fixture.body !== player,   // خودمان را نادیده بگیر
  );
  return hit !== null;
}

function update(input) {
  const v = player.linearVelocity;

  // افقی: سرعت را مستقیم تنظیم کن تا کنترل دقیق و قابل پیش‌بینی باشد.
  // عمودی را به فیزیک بسپار تا گرانش، شیب و ضربه همچنان کار کنند.
  player.setLinearVelocity(input.moveX * 6, Scalar.toFloat(v.y));

  if (input.jump && isGrounded()) {
    player.applyLinearImpulse(0, 9 * Scalar.toFloat(player.mass));
  }
}
```

<div dir="rtl">

**چرا کپسول؟** از پله بالا می‌رود و روی دیوار می‌لغزد بدون گیر کردن به گوشه‌ها
که مشکل جعبه است، و در عین حال یک شکل محدب ساده می‌ماند.

**چرا تنظیم سرعت به‌جای اعمال نیرو؟** حرکت مبتنی بر نیرو شناور حس می‌شود و به
جرم و اصطکاک وابسته است. تنظیم سرعت پاسخ آنی و قابل تنظیم می‌دهد — انتخاب
استاندارد بازی‌های پلتفرمر.

تغییرات ممکن:

- **کنترل هوایی:** وقتی `isGrounded()` نیست، `input.moveX` را در حدود ۰٫۳ ضرب کنید.
- **زمان کایوتی:** آخرین tickی که `isGrounded()` درست بوده را نگه دارید و تا حدود
  ۶ tick بعدش اجازهٔ پرش بدهید.
- **مدیریت شیب:** از `normal` پرتو استفاده کنید تا شیب‌های تندتر از حد مجاز را رد
  کنید یا حرکت را در امتداد سطح تنظیم کنید.

> برای زمان کایوتی و بافر پرش از `world.tick` استفاده کنید، هرگز از `Date.now()`.

---

## ۲. سکوی یک‌طرفه

وقتی جسم رو به بالا از سکو رد می‌شود، تماس را در `preSolve` غیرفعال کنید.

</div>

```ts
const platform = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
const platformFixture = platform.addFixture({ shape: Polygon.box(3, 0.2) });

world.setListener({
  preSolve(e) {
    const isA = e.fixtureA === platformFixture;
    const isB = e.fixtureB === platformFixture;
    if (!isA && !isB) return;

    const other = isA ? e.fixtureB : e.fixtureA;

    // هر چیزی که رو به بالا حرکت می‌کند مستقیم رد شود.
    if (Scalar.toFloat(other.body.linearVelocity.y) > 0) {
      e.contact.setEnabled(false);
    }
  },
});
```

<div dir="rtl">

تأیید شده: توپی که با سرعت ۱۴ متر بر ثانیه به بالا پرتاب شود رد می‌شود، بعد
فرود می‌آید و روی `y = 0.5` می‌ایستد.

فراخوان `setEnabled(false)` فقط یک گام دوام دارد، پس هر فریم خودکار بازارزیابی
می‌شود.

برای ورودی **پایین آمدن از سکو**، وقتی بازیکن دکمهٔ پایین را نگه داشته هم تماس را
غیرفعال کنید:

</div>

```ts
if (other.body === player && input.dropThrough) e.contact.setEnabled(false);
```

<div dir="rtl">

---

## ۳. سکوی متحرک

### Kinematic — ساده و صلب

</div>

```ts
const platform = world.createBody({
  type: BodyType.Kinematic,
  position: { x: 0, y: 2 },
  linearVelocity: { x: 1, y: 0 },
});
platform.addFixture({ shape: Polygon.box(3, 0.25), friction: 1 });

// در دو سر مسیر گشت‌زنی برمی‌گردد.
function update() {
  const x = Scalar.toFloat(platform.getPosition().x);
  if (x > 5)  platform.setLinearVelocity(-1, 0);
  if (x < -5) platform.setLinearVelocity( 1, 0);
}
```

<div dir="rtl">

جسم kinematic هل می‌دهد ولی هرگز هل داده نمی‌شود. اصطکاک، سرنشین‌ها را با خودش
می‌برد.

### Motor joint — آگاه به برخورد

وقتی سکو باید **پشت مانع بایستد** به‌جای له کردن یا رد شدن از آن، از این استفاده
کنید.

</div>

```ts
const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 2 } });

const platform = world.createBody({
  type: BodyType.Dynamic,
  position: { x: 0, y: 2 },
  gravityScale: 0,              // مفصل نگهش می‌دارد
});
platform.addFixture({ shape: Polygon.box(1.5, 0.2), density: 5, friction: 1.5 });

const motor = world.createMotorJoint({
  bodyA: anchor, bodyB: platform,
  maxForce: 8000, maxTorque: 8000,
  correctionFactor: 0.2,        // هشدار زیر را ببینید
});

// هر tick هدف را از شمارندهٔ tick بگیرید (هرگز از زمان دیواری).
motor.setLinearOffset(Math.sin(world.tick / 60) * 3, 0);
```

<div dir="rtl">

> **وقتی سرنشین دارید `correctionFactor` را پایین بیاورید.** این پارامتر تعیین
> می‌کند چه کسری از خطای باقیمانده در هر گام اصلاح شود. اندازه‌گیری روی یک جاروب
> سینوسی ۳ متری: با مقدار `۰٫۵` سکو چنان تند به سمت هدفش می‌پرد که اصطکاک
> نمی‌رسد و سرنشین سُر می‌خورد؛ با `۰٫۲` سرنشین تمام مسیر را همراه می‌آید و سکو
> همچنان با دقت ۰٫۰۳ متر هدف را دنبال می‌کند. بالاتر یعنی سفت‌تر، پایین‌تر یعنی
> نرم‌تر.

---

## ۴. انفجار

یک ضربهٔ شعاعی با افت خطی اعمال کنید. اگر می‌خواهید دیوارها سپر شوند، برای هر
جسم یک ray cast اضافه کنید.

</div>

```ts
function explode(cx, cy, radius, power) {
  world.queryAABB(cx - radius, cy - radius, cx + radius, cy + radius, (fixture) => {
    const body = fixture.body;
    if (body.type !== BodyType.Dynamic) return true;

    const p = body.worldCenter;
    const dx = Scalar.toFloat(p.x) - cx;
    const dy = Scalar.toFloat(p.y) - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > radius || dist < 1e-6) return true;

    const falloff = 1 - dist / radius;
    body.applyLinearImpulse(
      (dx / dist) * power * falloff,
      (dy / dist) * power * falloff,
      Scalar.toFloat(p.x), Scalar.toFloat(p.y),   // در مرکز جرم
    );
    return true;
  });
}

explode(0, 0, 5, 60);
```

<div dir="rtl">

برای یک هل شعاعی تمیز ضربه را در **مرکز جرم** اعمال کنید، یا برای افزودن چرخش در
نزدیک‌ترین نقطهٔ سطح.

برای انسداد خط دید، از مرکز انفجار به هر جسم یک پرتو بفرستید و آن‌هایی را که
اولین برخوردشان چیز دیگری است رد کنید.

---

## ۵. نوار نقاله

مقدار `tangentSpeed` باعث می‌شود سطح اجسام در تماس را در امتداد مماسش بکشد — بدون
هیچ کدی در هر فریم.

</div>

```ts
const belt = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
belt.addFixture({
  shape: Polygon.box(10, 0.5),
  friction: 0.9,        // باید به‌قدر کافی زیاد باشد تا کشش منتقل شود
  tangentSpeed: 4,      // متر بر ثانیه در امتداد مماس تماس
});
```

<div dir="rtl">

تأیید شده: جعبه‌ای که در `x = -5` رها شود در ۲۰۰ گام به `x = +7.3` می‌رسد.

مقادیر منفی جهت را برعکس می‌کنند. همین ترفند تردمیل، جریان آب و پیاده‌روی متحرک
هم می‌سازد.

---

## ۶. یک خودروی ساده

یک شاسی به‌همراه دو چرخ موتوردار روی مفصل لولا.

</div>

```ts
const chassis = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1 } });
chassis.addFixture({ shape: Polygon.box(1.2, 0.3), density: 1 });

const motors = [];
for (const dx of [-0.8, 0.8]) {
  const wheel = world.createBody({ type: BodyType.Dynamic, position: { x: dx, y: 0.5 } });
  wheel.addFixture({ shape: Circle.of(0.35), density: 1, friction: 1.5 });  // چسبندگی

  motors.push(world.createRevoluteJointAt(chassis, wheel, dx, 0.5, {
    enableMotor: true,
    motorSpeed: -12,        // منفی = جلو (ساعت‌گرد)
    maxMotorForce: 60,      // سقف گشتاور — حد کشش هم هست
  }));
}

// رانندگی
function drive(throttle) {                 // بین ۱- تا ۱
  for (const m of motors) m.setMotorSpeed(-12 * throttle);
}
```

<div dir="rtl">

تأیید شده: در ۳۰۰ گام ۲۰ متر جلو می‌رود و سرپا می‌ماند.

- **`maxMotorForce`** عملاً قدرت موتور است. خیلی زیاد باشد چرخ‌ها هرز می‌چرخند؛
  خیلی کم باشد نمی‌تواند بالا برود.
- **اصطکاک چرخ** چسبندگی است. قبل از بالا بردن گشتاور، این را بالا ببرید.
- برای **سیستم تعلیق**، هر چرخ را روی یک `PrismaticJoint` با
  `enableSpring: true` سوار کنید، یا یک فنر `DistanceJoint` بین شاسی و چرخ
  بگذارید.

---

## ۷. ناحیهٔ تریگر

سنسور هم‌پوشانی را تشخیص می‌دهد و رویداد می‌فرستد ولی هیچ نیرویی اعمال نمی‌کند.

</div>

```ts
const zone = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
zone.addFixture({
  shape: Polygon.box(1, 1),
  isSensor: true,
  userData: { kind: 'checkpoint', id: 3 },
});

const occupants = new Set();

world.setListener({
  beginSensor({ fixtureA, fixtureB }) {
    const sensor = fixtureA.isSensor ? fixtureA : fixtureB;
    const other  = fixtureA.isSensor ? fixtureB : fixtureA;
    occupants.add(other.body);
    onEnter(sensor.userData, other.body);
  },
  endSensor({ fixtureA, fixtureB }) {
    const other = fixtureA.isSensor ? fixtureB : fixtureA;
    occupants.delete(other.body);
  },
});
```

<div dir="rtl">

سنسورها همچنان فیلترینگ برخورد را رعایت می‌کنند، پس با `mask` می‌توانید محدود
کنید چه چیزی آن‌ها را فعال کند.

> در بازی‌های rollback، callbackهای سنسور در بازشبیه‌سازی دوباره اجرا می‌شوند.
> نیت را ثبت کنید و فقط وقتی tick از `rollback.oldestTick` قدیمی‌تر شد یک بار
> عمل کنید.

---

## ۸. زمین از روی نقشهٔ ارتفاع

به‌جای قطعه‌های جدا از `ChainShape` استفاده کنید — رأس‌های شبح را وصل می‌کند تا
اجسام روی درزها گیر نکنند.

</div>

```ts
const ground = world.createBody({ type: BodyType.Static });

const points = [];
for (let x = -30; x <= 30; x += 2) {
  points.push(Vec2.of(x, heightAt(x)));
}

for (const segment of ChainShape.fromPoints(points)) {
  ground.addFixture({ shape: segment, friction: 0.8 });
}
```

<div dir="rtl">

تأیید شده: توپی که از ۶ متری رها شود می‌غلتد و روی سطح آرام می‌گیرد، و جعبه‌ای با
سرعت ۱۲ متر بر ثانیه روی زنجیرهٔ صاف می‌لغزد بدون افت قابل اندازه‌گیری در درزها.

> **جهت چرخش سمت جامد را تعیین می‌کند.** وجهِ *سمت چپ جهت حرکت* جامد است، پس خطی
> که از چپ به راست نوشته شود (x افزایشی) از بالا جامد است. برای برعکس کردن ترتیب
> نقاط را معکوس کنید. یک حلقهٔ بستهٔ پادساعت‌گرد از داخل جامد است — اتاقی که
> نمی‌توانید از آن خارج شوید.

نقشهٔ ارتفاع را آن‌قدر متراکم نمونه‌برداری کنید که هیچ قطعه‌ای کوتاه‌تر از
کوچک‌ترین جسم دینامیک شما نشود.

---

## ۹. مفصل شکستنی

نیروی واکنش را بخوانید و بالاتر از یک آستانه مفصل را حذف کنید.

</div>

```ts
const MAX_FORCE = 400;                     // نیوتن
const reaction = Vec2.zero();

function checkBreak(joint) {
  // ضربه‌های مفصل در هر گام فرعی انباشته می‌شوند، پس ضریب تبدیل به نیوتن
  // مقدار `world.invSubStep` است — نه 1 / timeStep.
  joint.getReactionForce(reaction, world.invSubStep);
  if (Scalar.toFloat(reaction.length()) > MAX_FORCE) {
    world.destroyJoint(joint);
    spawnDebris(joint);
    return true;
  }
  return false;
}

// بعد از هر گام:
for (const joint of [...world.eachJoint()]) checkBreak(joint);
```

<div dir="rtl">

تأیید شده: یک بار ۳۲ کیلوگرمی دقیقاً وزن ۳۲۰ نیوتنی‌اش را از طریق مفصل گزارش
می‌کند؛ بالا بردن `MAX_FORCE` از آن مقدار بار را آویزان نگه می‌دارد و پایین آوردن
آن مفصل را می‌شکند و بار می‌افتد.

قبل از پیمایش از فهرست مفصل‌ها کپی بگیرید، چون `destroyJoint` آن را تغییر
می‌دهد. متد `getReactionTorque(invDt)` همین کار را برای خمش انجام می‌دهد.

برای سازه‌هایی که باید قبل از شکستن خم شوند، از `WeldJoint` در حالت نرم
(`linearHertz`، `angularHertz`) استفاده کنید و با همان معیار بشکنید.

---

## ۱۰. کشیدن با ماوس

</div>

```ts
let dragJoint = null;
const groundBody = world.createBody({ type: BodyType.Static });   // لنگرگاه مفصل

canvas.addEventListener('pointerdown', (ev) => {
  const { x, y } = debugDraw.screenToWorld(ev.offsetX, ev.offsetY);

  world.queryPoint(x, y, (fixture) => {
    if (fixture.body.type !== BodyType.Dynamic) return true;      // به گشتن ادامه بده
    dragJoint = world.createMouseJoint({
      bodyA: groundBody,
      bodyB: fixture.body,
      target: { x, y },
      hertz: 5, dampingRatio: 0.7,
      maxForce: 1000 * Scalar.toFloat(fixture.body.mass),
    });
    return false;                                                 // در اولین برخورد بایست
  });
});

canvas.addEventListener('pointermove', (ev) => {
  if (!dragJoint) return;
  const { x, y } = debugDraw.screenToWorld(ev.offsetX, ev.offsetY);
  dragJoint.setTarget(x, y);
});

canvas.addEventListener('pointerup', () => {
  if (dragJoint) { world.destroyJoint(dragJoint); dragJoint = null; }
});
```

<div dir="rtl">

ضرب کردن `maxForce` در جرم باعث می‌شود اجسام سنگین و سبک هر دو پاسخگو حس شوند.

> در مولتی‌پلیر، هدف درگ یک **ورودی** است: مثل هر ورودی دیگری کوانتیزه و ارسالش
> کنید، وگرنه همتاها واگرا می‌شوند.

---

## ۱۱. رندر نرم

فیزیک با نرخ ثابت اجرا می‌شود، نمایشگرها نه. با alpha بازگشتی از `accumulate`
بین دو وضعیت آخر درون‌یابی کنید.

</div>

```ts
const previous = new Map();     // body -> { x, y, angle }

function frame(now) {
  const dt = (now - last) / 1000;
  last = now;

  // وضعیت پیش از گام هر چیزی که رسم می‌کنید را ثبت کنید.
  for (const body of world.eachBody()) {
    const p = body.getPosition();
    previous.set(body, {
      x: Scalar.toFloat(p.x),
      y: Scalar.toFloat(p.y),
      angle: Scalar.toFloat(body.getAngle()),
    });
  }

  const alpha = world.accumulate(dt, 5);

  for (const body of world.eachBody()) {
    const prev = previous.get(body);
    const p = body.getPosition();
    const x = prev.x + (Scalar.toFloat(p.x) - prev.x) * alpha;
    const y = prev.y + (Scalar.toFloat(p.y) - prev.y) * alpha;
    drawSprite(body.userData.sprite, x, y);
  }

  requestAnimationFrame(frame);
}
```

<div dir="rtl">

زاویه‌ها را از کوتاه‌ترین مسیر درون‌یابی کنید، یا `Rot` را ذخیره و از
`Rot.nlerpTo` استفاده کنید، تا جسمی که از ±π عبور می‌کند در جهت اشتباه نچرخد.

---

## ۱۲. استخر اشیاء

فراخوان `destroyBody` رایگان نیست، و در بازی rollback تغییرات ساختاری به هر حال
باید قطعی باشند. بازیافت معمولاً بهتر است.

</div>

```ts
const pool = [];

function spawnBullet(x, y, vx, vy) {
  let body = pool.pop();
  if (body) {
    body.setTransform(x, y, 0);
    body.setLinearVelocity(vx, vy);
    body.setEnabled(true);                 // پروکسی‌های فاز گسترده دوباره درج می‌شوند
  } else {
    body = world.createBody({
      type: BodyType.Dynamic,
      position: { x, y },
      linearVelocity: { x: vx, y: vy },
      bullet: true,                        // برخورد پیوسته
    });
    body.addFixture({ shape: Circle.of(0.05), density: 5 });
  }
  return body;
}

function despawnBullet(body) {
  body.setEnabled(false);                  // منجمد و خارج از برخورد
  body.setLinearVelocity(0, 0);
  pool.push(body);
}
```

<div dir="rtl">

جسم غیرفعال کاملاً از حل‌کننده کنار گذاشته می‌شود و پروکسی‌هایش آزاد می‌شوند، پس
یک استخر بزرگ تقریباً هیچ هزینه‌ای ندارد. ترنسفرم را **قبل** از فعال‌سازی مجدد
تنظیم کنید.

---

## نکات کارایی

- **شکل‌ها را به اشتراک بگذارید.** یک نمونهٔ `Polygon` می‌تواند پشتیبان هزار
  فیکسچر باشد.
- **بگذارید اجسام بخوابند.** دنیای آرام‌گرفتهٔ ۱۰۰۰ جسمی ۱٫۴ میلی‌ثانیه در هر
  گام هزینه دارد در برابر ۸٫۰ میلی‌ثانیه در حالت فعال — تقریباً ۶ برابر
  ارزان‌تر. بی‌دلیل اجسام را بیدار نکنید.
- **`bullet` را فقط جایی که لازم است.** تماس‌های احتمالی از پس اجسام با سرعت
  متوسط برمی‌آیند؛ جاروب کردن همه‌چیز اتلاف است.
- **`subSteps` را بر `velocityIterations` ترجیح دهید** وقتی پشته باید سفت‌تر شود.
- **اندازه‌ها را در بازهٔ ۰٫۱ تا ۱۰ متر نگه دارید.** ثابت‌های تنظیم بر همین فرض
  استوارند. مختصات پیکسلی را قبل از رسیدن به موتور کوچک کنید.
- **قبل از تست‌های دقیق از `queryAABB` استفاده کنید.** فاز گسترده تقریباً
  همه‌چیز را در `O(log n)` رد می‌کند.

</div>

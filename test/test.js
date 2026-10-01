// 計算部分の検証。実行: node test/test.js
'use strict';
const T = require('../turnsim.js');

let fails = 0;
function check(cond, msg) {
  if (!cond) { fails++; console.log('  FAIL: ' + msg); }
}
const near = (a, b, tol) => Math.abs(a - b) <= tol;

for (const size of Object.keys(T.SIZES)) {
  const C = T.SIZES[size].cell;
  const v = size === 'classic' ? 2000 : 1300;

  // 1. 幾何: 開始と終了の角度差がターン角度、終点は区画中心か壁の中点
  for (const t of T.TURNS) {
    const g = T.geometry(t, C);
    const d = ((g.end[2] - g.start[2]) % 360 + 360) % 360;
    check(near(d, g.angle % 360, 1e-9), `${size} ${t}: 角度差 ${d}`);
    const e = T.idealLocal(t, C);
    const back = T.toGlobal(t, C, [e[0], e[1]]);
    check(near(back[0], g.end[0], 1e-9) && near(back[1], g.end[1], 1e-9), `${size} ${t}: 座標変換`);
  }

  // 2. 角加速度が無限大なら円弧。半径が合えばオフセットは0
  for (const [t, r] of [['big90', C], ['v90', C / Math.SQRT2], ['big180', C / 2]]) {
    const w = v / r * 180 / Math.PI;
    const res = T.evaluate(t, C, { v, wMax: w, wAc: 1e13, st: 0, end: 0 }, 0);
    check(near(res.along, 0, 0.1) && near(res.lat, 0, 0.1),
      `${size} ${t}: 半径${r.toFixed(1)}の円弧で終点がずれる (${res.along.toFixed(2)}, ${res.lat.toFixed(2)})`);
  }

  // 3. 解いたオフセットで走ると理想の終点に乗る(滑りあり)
  for (const t of T.TURNS) {
    if (t === 'big180') continue;
    const p = { v, wMax: 1500 * v / 2000 * 180 / C, wAc: 60000, st: 0, end: 0 };
    const sol = T.solveOffsets(t, C, p, 0.1);
    const res = T.evaluate(t, C, { ...p, st: sol.st, end: sol.end }, 0.1);
    check(near(res.along, 0, 0.1) && near(res.lat, 0, 0.1),
      `${size} ${t}: 解いたオフセットでずれる (${res.along.toFixed(2)}, ${res.lat.toFixed(2)})`);
  }

  // 4. 実測のずれからのオフセット修正で、ずれが消える(前後・横とも)
  for (const t of T.TURNS) {
    if (t === 'big180') continue;
    const p = { v, wMax: 1200 * 180 / C * v / 2000, wAc: 80000, st: 20, end: 30 };
    const r0 = T.evaluate(t, C, p, 0.12);
    const q = T.correctOffsets(t, C, p, r0.along, r0.lat);
    const r1 = T.evaluate(t, C, { ...p, st: q.st, end: q.end }, 0.12);
    check(near(r1.along, 0, 0.05) && near(r1.lat, 0, 0.05),
      `${size} ${t}: 修正後もずれる (${r1.along.toFixed(2)}, ${r1.lat.toFixed(2)})`);
  }

  // 5. 滑りで外側(横がマイナス)にずれる
  {
    const p = { v, wMax: 1500, wAc: 60000, st: 10, end: 10 };
    const a = T.evaluate('big90', C, p, 0), b = T.evaluate('big90', C, p, 0.2);
    check(b.lat < a.lat, `${size}: 滑りの向きが逆 (${a.lat.toFixed(2)} -> ${b.lat.toFixed(2)})`);
  }

  // 6. fitK は k を復元する
  {
    const p = { v, wMax: 1500, wAc: 60000, st: 10, end: 40 };
    const lat = T.evaluate('in45', C, p, 0.137, { dt: 1e-4 }).lat;
    const k = T.fitK('in45', C, p, lat);
    check(k !== null && near(k, 0.137, 1e-4), `${size}: fitK = ${k}`);
  }

  // 7. design の結果は条件を満たし、理想の線に乗る
  for (const t of T.TURNS) {
    const c = { v, k: 0.1, minSt: C * 0.1, minEnd: C * 0.03, wacMax: 200000 };
    const b = T.design(t, C, c);
    check(b !== null, `${size} ${t}: design が解なし`);
    if (!b) continue;
    check(b.st >= c.minSt - 0.5 && b.end >= c.minEnd - 0.5 && b.wAc <= c.wacMax,
      `${size} ${t}: 条件違反 st=${b.st.toFixed(1)} end=${b.end.toFixed(1)}`);
    const r = T.evaluate(t, C, b, c.k);
    const tol = t === 'big180' ? C / 180 + 0.2 : 0.2;
    check(near(r.along, 0, 0.2) && Math.abs(r.lat) <= tol,
      `${size} ${t}: design の結果がずれる (${r.along.toFixed(2)}, ${r.lat.toFixed(2)})`);
  }
}

// 8. ハーフはクラシックの縮小: 速度を半分にすると軌跡も半分
{
  const p = { v: 2000, wMax: 1500, wAc: 60000, st: 30, end: 40 };
  const a = T.evaluate('in45', 180, p, 0);
  const b = T.evaluate('in45', 90, { ...p, v: 1000, st: 15, end: 20 }, 0);
  check(near(a.along / 2, b.along, 0.05) && near(a.lat / 2, b.lat, 0.05), 'ハーフの縮小');
}

// 9. 三角形の角速度でも指定どおりの角度を回る
{
  const pr = T.omegaProfile(45, 3000, 50000);
  let s = 0; const n = 20000;
  for (let i = 0; i < n; i++) s += pr.w((i + 0.5) * pr.tEnd / n) * pr.tEnd / n;
  check(near(s, 45, 0.01) && pr.peak < 3000, `三角形の積分 ${s}`);
}

// 10. 速度の換算: 滑りなしなら同じ軌跡
{
  const p = { v: 2000, wMax: 1500, wAc: 60000, st: 30, end: 40 };
  const a = T.evaluate('in135', 180, p, 0), b = T.evaluate('in135', 180, T.scaleSpeed(p, 2600), 0);
  check(near(a.along, b.along, 0.05) && near(a.lat, b.lat, 0.05), '速度の換算');
}

// 11. ターン中の速度変化: vMid=v は速度一定と同じ。vMid<v で前後加速度が出て、横加速度は下がる
{
  const p = { v: 2000, wMax: 1500, wAc: 60000, st: 30, end: 40 };
  const a = T.evaluate('big90', 180, p, 0.1), b = T.evaluate('big90', 180, { ...p, vMid: 2000 }, 0.1);
  check(a.x === b.x && a.y === b.y && b.aLong === 0, 'vMid=v が速度一定と一致しない');
  const c = T.evaluate('big90', 180, { ...p, vMid: 1600 }, 0.1);
  check(c.aLong > 0 && c.aLat < a.aLat && c.aTot >= c.aLat, `vMid<v の加速度 ${c.aLong}, ${c.aLat}`);
  // 横加速度のピークは ω が最大のとき v=vMid: vMid*ω
  const pk = T.omegaProfile(90, 1500, 60000).peak;
  check(near(c.aLat, 1.6 * pk * Math.PI / 180, 0.05), `vMid のときの横加速度 ${c.aLat}`);
}

// 11b. ターン中に減速しても、解いたオフセットで理想の線に乗る
for (const t of ['big90', 'in45', 'v90', 'out135']) {
  const p = { v: 1800, vMid: 1400, wMax: 1300, wAc: 120000, st: 0, end: 0 };
  const sol = T.solveOffsets(t, 180, p, 0.1);
  const r = T.evaluate(t, 180, { ...p, st: sol.st, end: sol.end }, 0.1);
  check(near(r.along, 0, 0.1) && near(r.lat, 0, 0.1), `${t}: 減速ありのオフセットでずれる (${r.along.toFixed(2)}, ${r.lat.toFixed(2)})`);
  const q = T.correctOffsets(t, 180, { ...p, st: 20, end: 30 }, 3, -2);
  check(isFinite(q.st) && isFinite(q.end), `${t}: 減速ありの修正`);
}

// 12. 180度は角速度の二分法で横ずれ0になる(低速でも)
for (const size of Object.keys(T.SIZES)) {
  const C = T.SIZES[size].cell;
  for (const v of [400, 1000, 2000]) {
    const d = T.design('big180', C, { v, k: 0.1, minSt: C * 0.2, minEnd: C * 0.03, wacMax: 200000 });
    check(d !== null, `${size} big180 v=${v}: 解なし`);
    if (!d) continue;
    const r = T.evaluate('big180', C, d, 0.1);
    check(near(r.along, 0, 0.1) && near(r.lat, 0, 0.1), `${size} big180 v=${v}: ずれ (${r.along.toFixed(2)}, ${r.lat.toFixed(2)})`);
  }
}

// 13. maxSpeed: 見つけた速度は上限以内で、少し速くすると上限を超える(作れない)
{
  const C = 180, base = { k: 0.1, minSt: 37, minEnd: 5, wacMax: 130000 };
  const ref = T.design('in45', C, { ...base, v: 1800 });
  const aLim = T.evaluate('in45', C, ref, 0.1).aTot;
  for (const t of ['in45', 'big90', 'big180']) {
    const m = T.maxSpeed(t, C, { ...base, aLim, vLo: 500, vHi: 5000, step: 10 });
    check(m && m.aTot <= aLim + 1e-6, `${t}: maxSpeed が上限超え`);
    if (!m) continue;
    const d = T.design(t, C, { ...base, v: m.v + 20 });
    const over = !d || T.evaluate(t, C, d, 0.1).aTot > aLim;
    check(over, `${t}: ${m.v}+20 でも作れる(最高速度になっていない)`);
    if (t === 'in45') check(Math.abs(m.v - 1800) <= 10, `in45 自身の最高速度 ${m.v} が基準の 1800 と違う`);
  }
}

// 14. maxSpeedSearch を1回ずつ進めても maxSpeed と同じ答え。進み具合は増えていき、最後に1
{
  const c = { k: 0.1, minSt: 37, minEnd: 5, wacMax: 130000, aLim: 45, vLo: 500, vHi: 5000, step: 10 };
  const s = T.maxSpeedSearch('big90', 180, c);
  let prev = 0, steps = 0, mono = true;
  for (;;) { const r = s.next(); steps++; const f = s.frac(); if (f < prev) mono = false; prev = f; if (r.done) break; }
  const m = T.maxSpeed('big90', 180, c);
  check(s.result() && m && s.result().v === m.v, 'maxSpeedSearch と maxSpeed が違う');
  check(mono && s.frac() === 1 && steps <= s.total + 1, `進み具合 steps=${steps} total=${s.total}`);
  const none = T.maxSpeedSearch('in45', 180, { ...c, aLim: 1 });
  while (!none.next().done);
  check(none.result() === null, '作れないときは null');
}

console.log(fails ? `${fails}件の失敗` : 'OK');
process.exit(fails ? 1 : 0);

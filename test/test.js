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

console.log(fails ? `${fails}件の失敗` : 'OK');
process.exit(fails ? 1 : 0);

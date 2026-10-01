/*
 * マイクロマウスのターンシミュレーター(計算部分)
 *
 * ブラウザでは window.TurnSim、Node では require('./turnsim.js') で使う。
 *
 * モデル
 *  - 重心速度は入口・出口が v で、ターン中は角速度に連動して変えられる
 *        v(t) = v - (v - vMid) * ω(t) / ω_peak
 *    (角速度が最大のときに最低速度 vMid。vMid を省略すると v のまま一定)
 *  - 角速度 ω(t) は cos 型の加速 -> 等角速度 -> cos 型の減速
 *    (角度が足りないときは等角速度なしの三角形。ピーク角速度は自動で下がる)
 *  - 機体の向きはジャイロ制御で ω(t) どおりに回る
 *  - タイヤの横滑り: 進む向きが機体の向きから旋回の外側へスリップ角 β だけずれる
 *        β[deg] = k * a_lat[m/s^2]   a_lat = v * ω (横加速度)
 *    (スリップ角は遠心力 m*v*ω に比例するというモデル。
 *     速度が違うターンでも同じ k が使えるよう v*ω で持つ)
 *  - 前オフセット・後オフセットは、理想のターン開始点からの距離として走る
 *
 * 座標は迷路の座標[mm]。区画中心が C の整数倍、壁は ±C/2 の位置。
 * 左ターンだけを計算する(右ターンは鏡像)。
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.TurnSim = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  var D2R = Math.PI / 180;

  var SIZES = {
    classic: { label: 'クラシック', cell: 180 },
    half: { label: 'ハーフ', cell: 90 }
  };

  var TURNS = ['in45', 'v90', 'in135', 'out45', 'out135', 'big90', 'big180'];
  var TURN_NAMES = {
    big90: '大回り90', big180: '大回り180', in45: '斜め入り45', in135: '斜め入り135',
    out45: '斜め出45', out135: '斜め出135', v90: 'V90'
  };

  /* ターンの開始と理想の終了(迷路座標)。C は区画の大きさ。
   * 大回り・斜め入りは区画中心から、斜め出・V90 は斜めの壁の中点から始まる */
  function geometry(turn, C) {
    var h = C / 2;
    switch (turn) {
      case 'big90': return { start: [0, 0, 90], end: [-C, C, 180], angle: 90 };
      case 'big180': return { start: [0, 0, 90], end: [-C, 0, 270], angle: 180 };
      case 'in45': return { start: [0, 0, 90], end: [-h, C, 135], angle: 45 };
      case 'in135': return { start: [0, 0, 90], end: [-C, h, 225], angle: 135 };
      case 'out45': return { start: [h, C, 225], end: [0, 0, 270], angle: 45 };
      case 'out135': return { start: [C, h, 135], end: [0, 0, 270], angle: 135 };
      case 'v90': return { start: [h, 0, 45], end: [h, C, 135], angle: 90 };
    }
    throw new Error('unknown turn: ' + turn);
  }

  /* 区画端(ターンに入る区画の境界)から理想のターン開始点までの距離。
   * 大回り・斜め入りは区画中心から始まるので半区画、斜め出・V90 は境界上から始まるので 0。
   * 前距離(区画端から曲がり始めるまで) = st + edgeOffset */
  function edgeOffset(turn, C) {
    return /^(out|v90)/.test(turn) ? 0 : C / 2;
  }

  /* 開始姿勢を原点・前を+x・左を+yとした座標での理想の終点 [前, 左, 角度] */
  function idealLocal(turn, C) {
    var g = geometry(turn, C);
    var a = g.start[2] * D2R;
    var dx = g.end[0] - g.start[0], dy = g.end[1] - g.start[1];
    return [dx * Math.cos(a) + dy * Math.sin(a), -dx * Math.sin(a) + dy * Math.cos(a), g.angle];
  }

  function toGlobal(turn, C, p) {
    var g = geometry(turn, C);
    var a = g.start[2] * D2R;
    return [g.start[0] + p[0] * Math.cos(a) - p[1] * Math.sin(a),
            g.start[1] + p[0] * Math.sin(a) + p[1] * Math.cos(a)];
  }

  /* 角速度のプロファイル。angle[deg], wMax[deg/s], wAc[deg/s^2] */
  function omegaProfile(angle, wMax, wAc) {
    var x13 = Math.PI / (2 * wAc) * wMax * wMax; // 加速+減速で回る角度
    if (x13 > angle) {
      wMax = Math.sqrt(2 * wAc * angle / Math.PI);
      x13 = angle;
    }
    var t1 = Math.PI * wMax / (2 * wAc);
    var tc = (angle - x13) / wMax;
    return {
      peak: wMax,
      tEnd: 2 * t1 + tc,
      w: function (t) {
        if (t < t1) return wMax / 2 * (1 - Math.cos(Math.PI * t / t1));
        if (t < t1 + tc) return wMax;
        var tt = t - t1 - tc;
        if (tt < t1) return wMax / 2 * (1 + Math.cos(Math.PI * tt / t1));
        return 0;
      },
      /* 角加速度 [deg/s^2] */
      dw: function (t) {
        if (t < t1) return wAc * Math.sin(Math.PI * t / t1);
        if (t < t1 + tc) return 0;
        var tt = t - t1 - tc;
        if (tt < t1) return -wAc * Math.sin(Math.PI * tt / t1);
        return 0;
      }
    };
  }

  /* 左ターンを走らせる。p = {v, vMid(省略可), wMax, wAc, st, end}, angle[deg], k
   * 戻り値は開始姿勢基準の座標と、
   *   aLat: ピーク横加速度, aLong: ピーク前後加速度(絶対値), aTot: ピーク合成加速度(摩擦円) [m/s^2]
   *   peak: ピーク角速度, tTurn: 旋回の時間, time: 理想の開始点から終了点までの時間 [s]
   *   series: opt.series のとき [t, v, aLat, aLong, aTot] の列 */
  function runTurn(p, angle, k, opt) {
    opt = opt || {};
    var dt = opt.dt || 1e-5;
    var trace = opt.trace ? [[0, 0]] : null;
    var x = p.st, y = 0, th = 0;
    if (trace) trace.push([x, y]);
    var prof = omegaProfile(angle, p.wMax, p.wAc);
    var n = Math.max(1, Math.ceil(prof.tEnd / dt));
    var h = prof.tEnd / n;
    var v0 = p.v, dv = v0 - (p.vMid == null ? v0 : p.vMid);
    var series = opt.series ? [] : null;
    var every = Math.max(1, Math.floor(n / 200));
    var aLat = 0, aLong = 0, aTot = 0;
    for (var i = 0; i < n; i++) {
      var tm = (i + 0.5) * h;
      var wd = prof.w(tm);
      var v = v0 - dv * wd / prof.peak;
      var al = v / 1000 * wd * D2R;                       // 横加速度 [m/s^2]
      var ax = -dv * prof.dw(tm) / prof.peak / 1000;       // 前後加速度 [m/s^2]
      var at = Math.sqrt(al * al + ax * ax);
      if (al > aLat) aLat = al;
      if (Math.abs(ax) > aLong) aLong = Math.abs(ax);
      if (at > aTot) aTot = at;
      var thm = th + wd * D2R * h / 2;
      var beta = k * al * D2R; // k[deg/(m/s^2)] * a_lat -> rad
      var phi = thm - beta; // 外側(右)にずれる
      x += v * Math.cos(phi) * h;
      y += v * Math.sin(phi) * h;
      th += wd * D2R * h;
      if (trace && i % every === 0) trace.push([x, y]);
      if (series && i % every === 0) series.push([tm, v, al, ax, at]);
    }
    th = angle * D2R; // 角度はジャイロ制御で合う
    if (trace) trace.push([x, y]);
    x += p.end * Math.cos(th);
    y += p.end * Math.sin(th);
    if (trace) trace.push([x, y]);
    return {
      x: x, y: y, aLat: aLat, aLong: aLong, aTot: aTot, peak: prof.peak, trace: trace, series: series,
      tTurn: prof.tEnd, time: (p.st + p.end) / v0 + prof.tEnd
    };
  }

  /* 理想の終点からのずれを、出口の向きの [前後, 横] で返す(横は旋回の内側が+) */
  function exitError(turn, C, x, y) {
    var e = idealLocal(turn, C);
    var dx = x - e[0], dy = y - e[1];
    var a = e[2] * D2R;
    return [dx * Math.cos(a) + dy * Math.sin(a), -dx * Math.sin(a) + dy * Math.cos(a)];
  }

  function evaluate(turn, C, p, k, opt) {
    var g = geometry(turn, C);
    var r = runTurn(p, g.angle, k, opt);
    var e = exitError(turn, C, r.x, r.y);
    r.along = e[0];
    r.lat = e[1];
    return r;
  }

  /* 終点が理想の線に乗る前後オフセットを解く。180度は前後が平行で解けない
   * (横ずれは角速度で合わせる)ので、前オフセットを st180 に固定して後ろを解く */
  function solveOffsets(turn, C, p, k, opt) {
    var g = geometry(turn, C);
    var e = idealLocal(turn, C);
    var q = { v: p.v, vMid: p.vMid, wMax: p.wMax, wAc: p.wAc, st: 0, end: 0 };
    var r = runTurn(q, g.angle, k, opt);
    var a = e[2] * D2R;
    var s = Math.sin(a);
    if (Math.abs(s) < 1e-6) {
      var st = (opt && opt.st180 != null) ? opt.st180 : p.st;
      return { st: st, end: st + r.x - e[0], latErr: r.y - e[1] };
    }
    var end = (e[1] - r.y) / s;
    return { st: (e[0] - r.x) - end * Math.cos(a), end: end, latErr: 0 };
  }

  /* 実測の終点のずれ(出口の向きの前後・横)を前後オフセットの修正に直す */
  function correctOffsets(turn, C, p, along, lat) {
    var e = idealLocal(turn, C);
    var a = e[2] * D2R;
    // ずれ(出口座標) -> 開始座標
    var dx = along * Math.cos(a) - lat * Math.sin(a);
    var dy = along * Math.sin(a) + lat * Math.cos(a);
    var s = Math.sin(a);
    if (Math.abs(s) < 1e-6) {
      // 180度: 横は直せない。前後は後オフセットで直す(出口は逆向き)
      return { st: p.st, end: p.end - along, latLeft: lat };
    }
    var dEnd = -dy / s;
    var dSt = -dx - dEnd * Math.cos(a);
    return { st: p.st + dSt, end: p.end + dEnd, latLeft: 0 };
  }

  /* 横加速度が最小になる角速度・角加速度と前後オフセットを探す。
   * c = {v, k, minSt, minEnd, wacMax} */
  function design(turn, C, c) {
    var g = geometry(turn, C);
    var e = idealLocal(turn, C);
    var best = null;
    var wStep = 25, wacStep = 5000;
    var wTop = 12000;
    if (Math.abs(Math.sin(e[2] * D2R)) < 1e-6) return design180(turn, C, c);
    for (var w = 100; w <= wTop; w += wStep) {
      // ピーク横加速度は v*ω で決まる。すでに最良より大きければ打ち切り
      if (best && (c.v / 1000) * w * D2R > best.aLat + 1e-9) break;
      for (var wac = wacStep; wac <= c.wacMax + 1e-9; wac += wacStep) {
        var p = { v: c.v, wMax: w, wAc: wac, st: 0, end: 0 };
        var sol;
        if (Math.abs(Math.sin(e[2] * D2R)) < 1e-6) {
          sol = solveOffsets(turn, C, p, c.k, { dt: 2e-4, st180: c.minSt });
          if (Math.abs(sol.latErr) > C / 180) continue;
          if (sol.end < c.minEnd) {
            sol.st += c.minEnd - sol.end;
            sol.end = c.minEnd;
          }
        } else {
          sol = solveOffsets(turn, C, p, c.k, { dt: 2e-4 });
        }
        if (sol.st < c.minSt - 1e-6 || sol.end < c.minEnd - 1e-6) continue;
        var prof = omegaProfile(g.angle, w, wac);
        var aLat = (c.v / 1000) * prof.peak * D2R;
        if (!best || aLat < best.aLat - 1e-9) {
          best = { v: c.v, wMax: w, wAc: wac, st: sol.st, end: sol.end, aLat: aLat, peak: prof.peak };
        }
      }
    }
    if (!best) return null;
    // 細かい刻みで解き直す
    var fin = solveOffsets(turn, C, best, c.k, { st180: best.st });
    best.st = fin.st;
    best.end = fin.end;
    return best;
  }

  /* 180度: 横の位置は角速度だけで決まる(前後のオフセットは平行で効かない)ので、
   * 角加速度ごとに横ずれが0になる角速度を二分法で求め、横加速度が最小のものを取る */
  function design180(turn, C, c) {
    var g = geometry(turn, C);
    var e = idealLocal(turn, C);
    function lat(w, wac) {
      return runTurn({ v: c.v, wMax: w, wAc: wac, st: 0, end: 0 }, g.angle, c.k, { dt: 2e-4 }).y - e[1];
    }
    var best = null;
    for (var wac = 5000; wac <= c.wacMax + 1e-9; wac += 5000) {
      // 角速度を上げると旋回半径が小さくなり、横の移動量が減る
      var lo = 50, hi = 20000;
      if (lat(lo, wac) < 0 || lat(hi, wac) > 0) continue;
      for (var i = 0; i < 40; i++) {
        var mid = (lo + hi) / 2;
        if (lat(mid, wac) > 0) lo = mid; else hi = mid;
      }
      var w = (lo + hi) / 2;
      var p = { v: c.v, wMax: w, wAc: wac, st: c.minSt, end: 0 };
      var r = runTurn(p, g.angle, c.k, { dt: 2e-4 });
      if (Math.abs(r.y - e[1]) > 0.5) continue; // 三角形で頭打ちになり届かない
      var end = c.minSt + (r.x - c.minSt) - e[0];
      var st = c.minSt;
      if (end < c.minEnd) { st += c.minEnd - end; end = c.minEnd; }
      var prof = omegaProfile(g.angle, w, wac);
      var aLat = (c.v / 1000) * prof.peak * D2R;
      if (!best || aLat < best.aLat - 1e-9) {
        best = { v: c.v, wMax: w, wAc: wac, st: st, end: end, aLat: aLat, peak: prof.peak };
      }
    }
    if (!best) return null;
    var fin = solveOffsets(turn, C, best, c.k, { st180: best.st });
    best.st = fin.st;
    best.end = fin.end;
    return best;
  }

  /* 合成加速度が aLim 以下で作れる一番速い速度を探す(速度一定のターン、二分探索)。
   * c = {k, minSt, minEnd, wacMax, aLim, vLo, vHi, step}
   * 各速度で横加速度が最小になるパラメータ(design)を作り、それが aLim 以下なら作れる。
   * 画面を止めずに少しずつ進められるよう、1回の試行ごとに next() で進める形にしてある:
   *   var s = maxSpeedSearch(...); while (!s.next().done); s.result()
   * next() は {done, v(試した速度), ok(作れたか)} を返す。frac() は進み具合(0〜1) */
  function maxSpeedSearch(turn, C, c) {
    var aLim = c.aLim == null ? Infinity : c.aLim;
    function solve(v) {
      var d = design(turn, C, { v: v, k: c.k, minSt: c.minSt, minEnd: c.minEnd, wacMax: c.wacMax });
      if (!d) return null;
      var r = evaluate(turn, C, d, c.k, { dt: 1e-4 });
      if (r.aTot > aLim) return null;
      d.time = r.time; d.aTot = r.aTot;
      return d;
    }
    var step = c.step || 10;
    var lo = c.vLo, hi = c.vHi, best = null, phase = 0, done = false, result = null, n = 0;
    var total = 2 + Math.max(0, Math.ceil(Math.log(Math.max(1, (hi - lo) / step)) / Math.LN2));
    function finish(r) { done = true; result = r; }
    return {
      total: total,
      frac: function () { return done ? 1 : Math.min(0.99, n / total); },
      result: function () { return result; },
      next: function () {
        if (done) return { done: true };
        n++;
        var r;
        if (phase === 0) { // 下限で作れなければ解なし
          best = solve(lo);
          if (!best) finish(null); else phase = 1;
          return { done: done, v: lo, ok: !!best };
        }
        if (phase === 1) { // 上限で作れればそれが答え
          r = solve(hi);
          if (r) finish(r); else phase = 2;
          return { done: done, v: hi, ok: !!r };
        }
        var mid = Math.round((lo + hi) / 2 / step) * step;
        if (hi - lo <= step || mid <= lo || mid >= hi) { finish(best); return { done: true }; }
        r = solve(mid);
        if (r) { lo = mid; best = r; } else { hi = mid; }
        if (hi - lo <= step) finish(best);
        return { done: done, v: mid, ok: !!r };
      }
    };
  }

  function maxSpeed(turn, C, c) {
    var s = maxSpeedSearch(turn, C, c);
    while (!s.next().done) { /* 進める */ }
    return s.result();
  }

  /* 停止位置テストの横ずれ(実測、内側+)に合う k を求める */
  function fitK(turn, C, p, measuredLat) {
    function f(k) { return evaluate(turn, C, p, k, { dt: 1e-4 }).lat - measuredLat; }
    var lo = 0, hi = 0.5, flo = f(lo), fhi = f(hi);
    while (flo * fhi > 0 && hi < 20) { hi *= 2; fhi = f(hi); }
    if (flo * fhi > 0) return null;
    for (var i = 0; i < 60; i++) {
      var mid = (lo + hi) / 2, fm = f(mid);
      if (fm * flo > 0) { lo = mid; flo = fm; } else { hi = mid; }
    }
    return (lo + hi) / 2;
  }

  /* 速度を変えたときの換算(滑りがなければ同じ軌跡になる) */
  function scaleSpeed(p, v) {
    var s = v / p.v;
    var q = { v: v, wMax: p.wMax * s, wAc: p.wAc * s * s, st: p.st, end: p.end };
    if (p.vMid != null) q.vMid = p.vMid * s;
    return q;
  }

  function stats(values) {
    var n = values.length;
    if (!n) return null;
    var m = values.reduce(function (a, b) { return a + b; }, 0) / n;
    var sd = Math.sqrt(values.reduce(function (a, b) { return a + (b - m) * (b - m); }, 0) / (n > 1 ? n - 1 : 1));
    return { n: n, mean: m, sd: sd, min: Math.min.apply(null, values), max: Math.max.apply(null, values) };
  }

  return {
    SIZES: SIZES, TURNS: TURNS, TURN_NAMES: TURN_NAMES,
    geometry: geometry, edgeOffset: edgeOffset, idealLocal: idealLocal, toGlobal: toGlobal,
    omegaProfile: omegaProfile, runTurn: runTurn, exitError: exitError, evaluate: evaluate,
    solveOffsets: solveOffsets, correctOffsets: correctOffsets, design: design, maxSpeed: maxSpeed, maxSpeedSearch: maxSpeedSearch, fitK: fitK,
    scaleSpeed: scaleSpeed, stats: stats
  };
});

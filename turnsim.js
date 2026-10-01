/*
 * マイクロマウスのターンシミュレーター(計算部分)
 *
 * ブラウザでは window.TurnSim、Node では require('./turnsim.js') で使う。
 *
 * モデル
 *  - 並進速度 v はターン中一定
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
      }
    };
  }

  /* 左ターンを走らせる。p = {v, wMax, wAc, st, end}, angle[deg], k
   * 戻り値は開始姿勢基準の座標 {x, y, aLat, peak, trace} */
  function runTurn(p, angle, k, opt) {
    opt = opt || {};
    var dt = opt.dt || 1e-5;
    var trace = opt.trace ? [[0, 0]] : null;
    var x = p.st, y = 0, th = 0;
    if (trace) trace.push([x, y]);
    var prof = omegaProfile(angle, p.wMax, p.wAc);
    var n = Math.max(1, Math.ceil(prof.tEnd / dt));
    var h = prof.tEnd / n;
    var vm = p.v / 1000;
    var every = Math.max(1, Math.floor(n / 200));
    for (var i = 0; i < n; i++) {
      var wd = prof.w((i + 0.5) * h);
      var thm = th + wd * D2R * h / 2;
      var beta = k * vm * wd * D2R * D2R; // k[deg/(m/s^2)] * a_lat -> rad
      var phi = thm - beta; // 外側(右)にずれる
      x += p.v * Math.cos(phi) * h;
      y += p.v * Math.sin(phi) * h;
      th += wd * D2R * h;
      if (trace && i % every === 0) trace.push([x, y]);
    }
    th = angle * D2R; // 角度はジャイロ制御で合う
    if (trace) trace.push([x, y]);
    x += p.end * Math.cos(th);
    y += p.end * Math.sin(th);
    if (trace) trace.push([x, y]);
    return { x: x, y: y, aLat: vm * prof.peak * D2R, peak: prof.peak, trace: trace };
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
    var q = { v: p.v, wMax: p.wMax, wAc: p.wAc, st: 0, end: 0 };
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
    return { v: v, wMax: p.wMax * s, wAc: p.wAc * s * s, st: p.st, end: p.end };
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
    geometry: geometry, idealLocal: idealLocal, toGlobal: toGlobal,
    omegaProfile: omegaProfile, runTurn: runTurn, exitError: exitError, evaluate: evaluate,
    solveOffsets: solveOffsets, correctOffsets: correctOffsets, design: design, fitK: fitK,
    scaleSpeed: scaleSpeed, stats: stats
  };
});

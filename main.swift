// Stick Chase: a stick figure that parkours across your screen trying to catch the mouse cursor.
//
// Surfaces come from two sources: window frames (title bars are one-way ledges, sides are climbable
// walls) and, with Screen Recording permission, edge detection on the screen image so buttons, icons,
// images and panels become ledges and walls too. A route planner (Dijkstra over run / jump /
// ledge-grab / drop / climb moves) picks a route; a momentum-based controller runs it, steering in
// the air and grabbing ledges it comes up short on. The figure is animated from key poses through
// per-joint springs (follow-through and overlap) and drawn with Core Animation in one click-through
// overlay per screen.

import Cocoa
import QuartzCore
import ScreenCaptureKit

// MARK: - Tunables (screen points)

let S: CGFloat = 0.6                 // body units -> screen points (poses are authored in body units)
let GRAV: CGFloat = 1700
let RUN_SPEED: CGFloat = 205
let SPRINT_SPEED: CGFloat = 265
let GROUND_ACCEL: CGFloat = 1300
let SKID_DECEL: CGFloat = 1900
let AIR_ACCEL: CGFloat = 650
let AIR_VX_MAX: CGFloat = 330
let JUMP_MAX: CGFloat = 60           // highest a jump lifts his feet
let HAND_REACH: CGFloat = 36         // hand height above feet with arms raised
let CLIMB_SPEED: CGFloat = 170
let WALLRUN_VY: CGFloat = 380
let WALLRUN_GRAV: CGFloat = 1000
let GRAB_RADIUS: CGFloat = 13
let WALL_OFFSET: CGFloat = 6         // feet distance from a wall while on it
let MANTLE_IN: CGFloat = 8           // where he stands after pulling up onto a ledge
let NODE_STRIDE: CGFloat = 30
let HANG_LEN: CGFloat = 37           // hand-to-feet distance hanging from the cursor
let RIDE_FLING: CGFloat = 1700
let LEAP_RANGE: CGFloat = 260        // horizontal reach of a leap off a wall
let LEDGE_REACH: CGFloat = JUMP_MAX + HAND_REACH - 12
let WALL_GRAB_REACH: CGFloat = JUMP_MAX + HAND_REACH - 10
let LINE_W: CGFloat = 2.0
let VISION_OWNER: Int = -1000        // owners at or below this come from edge detection
let BUNDLE_ID = "com.natesute.stickchase"

// Body units
let LEG_U: CGFloat = 13, LEG_L: CGFloat = 13, ARM_U: CGFloat = 11, ARM_L: CGFloat = 11, HEAD_R: CGFloat = 6.5

// MARK: - Math

@inline(__always) func P(_ x: CGFloat, _ y: CGFloat) -> CGPoint { CGPoint(x: x, y: y) }
extension CGPoint {
    static func + (a: CGPoint, b: CGPoint) -> CGPoint { P(a.x + b.x, a.y + b.y) }
    static func - (a: CGPoint, b: CGPoint) -> CGPoint { P(a.x - b.x, a.y - b.y) }
    static func * (a: CGPoint, s: CGFloat) -> CGPoint { P(a.x * s, a.y * s) }
    var len: CGFloat { hypot(x, y) }
}
func lerp(_ a: CGFloat, _ b: CGFloat, _ t: CGFloat) -> CGFloat { a + (b - a) * t }
func lerp(_ a: CGPoint, _ b: CGPoint, _ t: CGFloat) -> CGPoint { P(lerp(a.x, b.x, t), lerp(a.y, b.y, t)) }
func clampf(_ v: CGFloat, _ lo: CGFloat, _ hi: CGFloat) -> CGFloat { min(max(v, lo), hi) }
func approach(_ v: CGFloat, _ target: CGFloat, _ step: CGFloat) -> CGFloat {
    v < target ? min(v + step, target) : max(v - step, target)
}
func angleDiff(_ a: CGFloat, _ b: CGFloat) -> CGFloat {
    var d = (b - a).truncatingRemainder(dividingBy: 2 * .pi)
    if d > .pi { d -= 2 * .pi }
    if d < -.pi { d += 2 * .pi }
    return d
}
func ease(_ t: CGFloat) -> CGFloat { let u = clampf(t, 0, 1); return u * u * (3 - 2 * u) }
func frac(_ x: CGFloat) -> CGFloat { x - floor(x) }
func rnd(_ r: ClosedRange<CGFloat>) -> CGFloat { CGFloat.random(in: r) }
func chance(_ p: CGFloat) -> Bool { CGFloat.random(in: 0...1) < p }
func clampLen(_ p: CGPoint, _ m: CGFloat) -> CGPoint { let l = p.len; return l > m ? p * (m / l) : p }
func segPointDist(_ a: CGPoint, _ b: CGPoint, _ p: CGPoint) -> CGFloat {
    let ab = b - a
    let l2 = ab.x * ab.x + ab.y * ab.y
    if l2 < 1e-6 { return (p - a).len }
    let t = clampf(((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2, 0, 1)
    return (a + ab * t - p).len
}
func catmull(_ p0: CGPoint, _ p1: CGPoint, _ p2: CGPoint, _ p3: CGPoint, _ t: CGFloat) -> CGPoint {
    let t2 = t * t, t3 = t2 * t
    return (p1 * 2 + (p2 - p0) * t + (p0 * 2 - p1 * 5 + p2 * 4 - p3) * t2 + (p1 * 3 - p0 - p2 * 3 + p3) * t3) * 0.5
}

struct JumpSol { var vx: CGFloat; var vy: CGFloat; var t: CGFloat; var apex: CGFloat }

/// Ballistic jump that travels (dx, dy) and lands while descending. Nil if out of range.
func solveJump(dx: CGFloat, dy: CGFloat) -> JumpSol? {
    var apex = dy > 0 ? dy + 14 : 10 + min(abs(dx) * 0.06, 25)
    while apex <= JUMP_MAX + 0.01 {
        let vy = (2 * GRAV * apex).squareRoot()
        let t = vy / GRAV + (2 * (apex - dy) / GRAV).squareRoot()
        let vx = dx / t
        if abs(vx) <= AIR_VX_MAX { return JumpSol(vx: vx, vy: vy, t: t, apex: apex) }
        if apex >= JUMP_MAX { break }
        apex = min(apex + 10, JUMP_MAX)
    }
    return nil
}

// MARK: - World geometry

struct Seg {
    var x0: CGFloat, x1: CGFloat, y: CGFloat, owner: Int   // window number, negative for floors, <= VISION_OWNER for detected edges
    func contains(_ x: CGFloat, _ m: CGFloat = 0) -> Bool { x >= x0 - m && x <= x1 + m }
}

struct Wall {
    var x: CGFloat, y0: CGFloat, y1: CGFloat
    var side: CGFloat        // +1: the solid thing is to the right of this edge; -1: to the left
    var owner: Int
    var hasTop: Bool         // there's a ledge at the top to pull up onto
    var baseX: CGFloat { x - side * WALL_OFFSET }
}

struct GNode { var seg: Int; var x: CGFloat }
enum EdgeKind { case walk, jump, ledge, drop, climb(Int) }
struct Edge { var to: Int; var cost: CGFloat; var kind: EdgeKind }

enum Action {
    case walk
    case jump(from: CGFloat, to: CGPoint, target: Seg, ledge: Bool)
    case drop(from: CGFloat)
    case climb(from: CGFloat, wall: Wall, leap: Bool)
    case catchJump
    case dropCatch(from: CGFloat)
    case frustrated(at: CGFloat)

    var isFrustrated: Bool { if case .frustrated = self { return true }; return false }
}

enum GoalKind { case catchJump, walkGrab, dropCatch, frustrated }

struct MinHeap {
    var a: [(CGFloat, Int)] = []
    var isEmpty: Bool { a.isEmpty }
    mutating func push(_ k: CGFloat, _ v: Int) {
        a.append((k, v))
        var i = a.count - 1
        while i > 0 {
            let p = (i - 1) / 2
            if a[p].0 <= a[i].0 { break }
            a.swapAt(p, i); i = p
        }
    }
    mutating func pop() -> (CGFloat, Int) {
        let top = a[0]
        let last = a.removeLast()
        if !a.isEmpty {
            a[0] = last
            var i = 0
            while true {
                let l = 2 * i + 1, r = l + 1
                var m = i
                if l < a.count && a[l].0 < a[m].0 { m = l }
                if r < a.count && a[r].0 < a[m].0 { m = r }
                if m == i { break }
                a.swapAt(i, m); i = m
            }
        }
        return top
    }
}

typealias Iv = (CGFloat, CGFloat)

func subtract(_ ivs: [Iv], _ a: CGFloat, _ b: CGFloat) -> [Iv] {
    var out: [Iv] = []
    for (lo, hi) in ivs {
        if b <= lo || a >= hi { out.append((lo, hi)); continue }
        if a > lo { out.append((lo, a)) }
        if b < hi { out.append((b, hi)) }
    }
    return out
}

func mergeIvs(_ ivs: [Iv]) -> [Iv] {
    var out: [Iv] = []
    for iv in ivs.sorted(by: { $0.0 < $1.0 }) {
        if let last = out.last, iv.0 <= last.1 + 0.5 { out[out.count - 1].1 = max(last.1, iv.1) } else { out.append(iv) }
    }
    return out
}

func sig(_ parts: Int...) -> Int {
    var h = Hasher()
    for p in parts { h.combine(p) }
    return h.finalize() | 1
}

typealias ScreenInfo = (frame: CGRect, visible: CGRect)

final class World {
    var rects: [Int: CGRect] = [:]
    var screens: [ScreenInfo] = []
    var segs: [Seg] = []
    var walls: [Wall] = []
    var nodes: [GNode] = []
    var adj: [[Edge]] = []
    var segNodes: [[Int]] = []
    var version = 0
    var buildMs: Double = 0
    var visionSegs: [Seg] = []
    var visionWalls: [Wall] = []
    private var byY: [Int] = []          // seg indices sorted by height
    private var ysSorted: [CGFloat] = []
    private var lastWins: [(Int, CGRect)] = []
    private var signature: [CGFloat] = []
    private let myPID = getpid()

    func refresh(force: Bool = false) {
        let scr = NSScreen.screens
        guard let primary = scr.first else { return }
        let h0 = primary.frame.maxY
        var wins: [(Int, CGRect)] = []
        if let list = CGWindowListCopyWindowInfo([.optionOnScreenOnly, .excludeDesktopElements], kCGNullWindowID) as? [[String: Any]] {
            for d in list {
                guard (d[kCGWindowLayer as String] as? Int ?? -1) == 0 else { continue }
                if let pid = d[kCGWindowOwnerPID as String] as? Int32, pid == myPID { continue }
                if let a = d[kCGWindowAlpha as String] as? Double, a < 0.05 { continue }
                guard let bd = d[kCGWindowBounds as String] as? NSDictionary,
                      let b = CGRect(dictionaryRepresentation: bd as CFDictionary) else { continue }
                if b.width < 80 || b.height < 50 { continue }
                let id = d[kCGWindowNumber as String] as? Int ?? 0
                wins.append((id, CGRect(x: b.minX, y: h0 - b.maxY, width: b.width, height: b.height)))
            }
        }
        // A full-screen app hides the Dock and menu bar, so the whole screen is usable there.
        let scrs = scr.map { sc -> ScreenInfo in
            let full = wins.contains { $0.1.minX <= sc.frame.minX + 2 && $0.1.maxX >= sc.frame.maxX - 2 &&
                                       abs($0.1.minY - sc.frame.minY) < 2 && $0.1.height >= sc.frame.height * 0.9 }
            return (frame: sc.frame, visible: full ? sc.frame : sc.visibleFrame)
        }
        var sg: [CGFloat] = []
        for (id, r) in wins { sg += [CGFloat(id), r.minX, r.minY, r.width, r.height] }
        for s in scrs { sg += [s.frame.minX, s.frame.minY, s.frame.width, s.frame.height, s.visible.minX, s.visible.minY, s.visible.width, s.visible.height] }
        if sg == signature && !force { return }
        signature = sg
        screens = scrs
        rects = [:]
        for (id, r) in wins { rects[id] = r }
        rebuild(wins)
    }

    /// New edges from the screen image.
    func setVision(segs: [Seg], walls: [Wall]) {
        visionSegs = segs
        visionWalls = walls
        rebuild(lastWins)
    }

    private func clipToScreens(_ ivs: [Iv], _ y: CGFloat) -> [Iv] {
        var out: [Iv] = []
        for (a, b) in ivs {
            for s in screens where y >= s.visible.minY && y <= s.frame.maxY + 1 {
                let lo = max(a, s.visible.minX), hi = min(b, s.visible.maxX)
                if hi > lo { out.append((lo, hi)) }
            }
        }
        return mergeIvs(out)
    }

    func screenIndex(at p: CGPoint) -> Int? {
        screens.firstIndex { p.x >= $0.visible.minX && p.x <= $0.visible.maxX && p.y >= $0.visible.minY - 2 && p.y <= $0.frame.maxY + 2 }
    }

    func nearestScreen(to p: CGPoint) -> Int {
        var best = 0
        var bd = CGFloat.infinity
        for (i, s) in screens.enumerated() {
            let v = s.visible
            let d = hypot(max(v.minX - p.x, 0, p.x - v.maxX), max(v.minY - p.y, 0, p.y - s.frame.maxY))
            if d < bd { bd = d; best = i }
        }
        return best
    }

    func rebuild(_ wins: [(Int, CGRect)]) {
        let t0 = CACurrentMediaTime()
        lastWins = wins
        var segs: [Seg] = []
        for (i, s) in screens.enumerated() {
            segs.append(Seg(x0: s.visible.minX, x1: s.visible.maxX, y: s.visible.minY, owner: -(i + 1)))
        }
        // Window tops: the visible part of each top edge (windows in front hide it).
        for i in wins.indices {
            let (id, r) = wins[i]
            let y = r.maxY
            var ivs: [Iv] = [(r.minX, r.maxX)]
            for j in 0..<i {
                let o = wins[j].1
                if o.minY < y - 1 && o.maxY > y - 1 { ivs = subtract(ivs, o.minX, o.maxX) }
            }
            for (a, b) in clipToScreens(ivs, y) where b - a >= 20 {
                segs.append(Seg(x0: a, x1: b, y: y, owner: id))
            }
        }
        // Detected edges, minus ones that duplicate a window edge or sit on the floor.
        let structural = segs
        for v in visionSegs {
            let dup = structural.contains { s in
                abs(s.y - v.y) <= 4 && min(s.x1, v.x1) - max(s.x0, v.x0) >= (v.x1 - v.x0) * 0.5
            }
            if !dup && screenIndex(at: P((v.x0 + v.x1) / 2, v.y)) != nil { segs.append(v) }
        }
        // Window sides: visible parts become climbable walls.
        var walls: [Wall] = []
        for i in wins.indices {
            let (id, r) = wins[i]
            for side: CGFloat in [1, -1] {
                let x = side > 0 ? r.minX : r.maxX
                let bx = x - side * WALL_OFFSET
                guard screens.contains(where: { bx > $0.visible.minX + 4 && bx < $0.visible.maxX - 4 && r.maxY > $0.visible.minY && r.minY < $0.frame.maxY }) else { continue }
                var ivs: [Iv] = [(r.minY, r.maxY)]
                for j in 0..<i {
                    let o = wins[j].1
                    if o.minX < x + 1 && o.maxX > x - 1 { ivs = subtract(ivs, o.minY, o.maxY) }
                }
                for (a, b) in ivs where b - a >= 24 {
                    walls.append(Wall(x: x, y0: a, y1: b, side: side, owner: id, hasTop: false))
                }
            }
        }
        let windowWalls = walls
        for v in visionWalls {
            let dup = windowWalls.contains { w in
                abs(w.x - v.x) <= 4 && min(w.y1, v.y1) - max(w.y0, v.y0) >= (v.y1 - v.y0) * 0.5
            }
            let bx = v.baseX
            if !dup && screens.contains(where: { bx > $0.visible.minX + 4 && bx < $0.visible.maxX - 4 }) { walls.append(v) }
        }
        // Screen edges are walls too (unless another screen continues past them).
        for (i, s) in screens.enumerated() {
            let v = s.visible
            let leftOpen = !screens.contains { abs($0.frame.maxX - s.frame.minX) < 2 && $0.frame.maxY > v.minY && $0.frame.minY < v.maxY }
            let rightOpen = !screens.contains { abs($0.frame.minX - s.frame.maxX) < 2 && $0.frame.maxY > v.minY && $0.frame.minY < v.maxY }
            if leftOpen { walls.append(Wall(x: v.minX, y0: v.minY, y1: v.maxY, side: -1, owner: -100 - 2 * i, hasTop: false)) }
            if rightOpen { walls.append(Wall(x: v.maxX, y0: v.minY, y1: v.maxY, side: 1, owner: -101 - 2 * i, hasTop: false)) }
        }
        self.segs = segs
        byY = segs.indices.sorted { segs[$0].y < segs[$1].y }
        ysSorted = byY.map { segs[$0].y }
        for i in walls.indices { walls[i].hasTop = topSeg(at: walls[i]) != nil }
        self.walls = walls
        buildGraph()
        buildMs = (CACurrentMediaTime() - t0) * 1000
        version += 1
    }

    /// Seg indices with lo <= y <= hi.
    func segsInY(_ lo: CGFloat, _ hi: CGFloat) -> ArraySlice<Int> {
        var a = 0, b = ysSorted.count
        while a < b { let m = (a + b) / 2; if ysSorted[m] < lo { a = m + 1 } else { b = m } }
        var c = a, d = ysSorted.count
        while c < d { let m = (c + d) / 2; if ysSorted[m] <= hi { c = m + 1 } else { d = m } }
        return byY[a..<c]
    }

    func segIndex(owner: Int, x: CGFloat, y: CGFloat) -> Int? {
        for i in segsInY(y - 2, y + 2) where segs[i].owner == owner && segs[i].contains(x, 3) { return i }
        return nil
    }

    /// Closest seg under a point, any owner (used to keep standing on detected edges between frames).
    func segNear(x: CGFloat, y: CGFloat, tol: CGFloat) -> Int? {
        var best: Int? = nil
        for i in segsInY(y - tol, y + tol) where segs[i].contains(x, 2) {
            if best == nil || abs(segs[i].y - y) < abs(segs[best!].y - y) { best = i }
        }
        return best
    }

    private func topSeg(at w: Wall) -> Int? {
        segNear(x: w.x + w.side * MANTLE_IN, y: w.y1, tol: 3)
    }

    func topSeg(of w: Wall) -> Int? { w.hasTop ? topSeg(at: w) : nil }

    func nearestNode(seg: Int, x: CGFloat) -> Int? {
        let list = segNodes[seg]
        if list.isEmpty { return nil }
        var lo = 0, hi = list.count - 1
        while lo < hi {
            let mid = (lo + hi) / 2
            if nodes[list[mid]].x < x { lo = mid + 1 } else { hi = mid }
        }
        var best = list[lo]
        if lo > 0 && abs(nodes[list[lo - 1]].x - x) < abs(nodes[best].x - x) { best = list[lo - 1] }
        return best
    }

    func climbBase(_ s: Seg, _ w: Wall) -> Bool {
        s.contains(w.baseX, -1) && s.y < w.y1 - 15 && s.y >= w.y0 - WALL_GRAB_REACH
    }

    /// True if a jump would come down on some platform other than the target before reaching it.
    func trajectoryBlocked(x0: CGFloat, y0: CGFloat, vx: CGFloat, vy: CGFloat, tLand: CGFloat, target: Seg) -> Bool {
        let apexY = y0 + vy * vy / (2 * GRAV)
        let landY = y0 + vy * tLand - 0.5 * GRAV * tLand * tLand
        for i in segsInY(min(landY, y0) - 1, apexY) {
            let s = segs[i]
            if s.owner == target.owner && s.y == target.y && s.x0 == target.x0 { continue }
            let disc = vy * vy - 2 * GRAV * (s.y - y0)
            if disc < 0 { continue }
            let tc = (vy + disc.squareRoot()) / GRAV
            if tc >= tLand - 0.005 || tc < 0.03 { continue }
            let xc = x0 + vx * tc
            if xc > s.x0 - 3 && xc < s.x1 + 3 { return true }
        }
        return false
    }

    private func buildGraph() {
        nodes = []
        segNodes = Array(repeating: [], count: segs.count)
        for (i, s) in segs.enumerated() {
            var xs: [CGFloat] = []
            let a = s.x0 + 5, b = s.x1 - 5
            if b <= a { xs = [(s.x0 + s.x1) / 2] } else {
                xs.append(a)
                var x = a + NODE_STRIDE
                while x < b - NODE_STRIDE * 0.5 { xs.append(x); x += NODE_STRIDE }
                xs.append(b)
            }
            for w in walls where climbBase(s, w) { xs.append(w.baseX) }
            xs.sort()
            var kept: [CGFloat] = []
            for x in xs where kept.last.map({ x - $0 >= 4 }) ?? true { kept.append(x) }
            for x in kept { segNodes[i].append(nodes.count); nodes.append(GNode(seg: i, x: x)) }
        }
        adj = Array(repeating: [], count: nodes.count)

        for list in segNodes where list.count > 1 {
            for k in 0..<(list.count - 1) {
                let a = list[k], b = list[k + 1]
                let c = abs(nodes[b].x - nodes[a].x) / RUN_SPEED
                adj[a].append(Edge(to: b, cost: c, kind: .walk))
                adj[b].append(Edge(to: a, cost: c, kind: .walk))
            }
        }

        for ai in nodes.indices {
            let a = nodes[ai]
            let p = segs[a.seg]
            for qi in segsInY(p.y - 450, p.y + LEDGE_REACH) where qi != a.seg {
                let q = segs[qi]
                let dy = q.y - p.y
                let tx = q.x1 - q.x0 > 14 ? clampf(a.x, q.x0 + 7, q.x1 - 7) : (q.x0 + q.x1) / 2
                if abs(tx - a.x) > 380 { continue }
                guard let bi = nearestNode(seg: qi, x: tx) else { continue }
                let dx = nodes[bi].x - a.x
                if abs(dx) < 4 && dy < 0 { continue }
                if dy <= JUMP_MAX - 14, let j = solveJump(dx: dx, dy: dy),
                   !trajectoryBlocked(x0: a.x, y0: p.y, vx: j.vx, vy: j.vy, tLand: j.t, target: q) {
                    adj[ai].append(Edge(to: bi, cost: j.t + 0.1 + (dy < -200 ? 0.3 : 0), kind: .jump))
                    continue
                }
                if dy > 10 {
                    // Too high to land on: jump, catch the edge with his hands and pull up.
                    let apex = min(JUMP_MAX, max(dy - HAND_REACH + 10, 12))
                    let tA = (2 * apex / GRAV).squareRoot()
                    if abs(dx) <= tA * AIR_VX_MAX * 0.95 {
                        adj[ai].append(Edge(to: bi, cost: tA + 0.5, kind: .ledge))
                    }
                }
            }
            // Drop through the platform (not the floor) to whatever is directly below.
            if p.owner >= 0 || p.owner <= VISION_OWNER {
                var best: Int? = nil
                for qi in segsInY(p.y - 2000, p.y - 2).reversed() where qi != a.seg && segs[qi].contains(a.x) { best = qi; break }
                if let qi = best, let bi = nearestNode(seg: qi, x: a.x), abs(nodes[bi].x - a.x) < 20 {
                    let dy = p.y - segs[qi].y
                    adj[ai].append(Edge(to: bi, cost: (2 * dy / GRAV).squareRoot() + 0.2 + (dy > 200 ? 0.3 : 0), kind: .drop))
                }
            }
        }

        for (wi, w) in walls.enumerated() {
            guard let top = topSeg(of: w) else { continue }
            for si in segsInY(w.y0 - WALL_GRAB_REACH, w.y1 - 15) where climbBase(segs[si], w) {
                guard let ai = nearestNode(seg: si, x: w.baseX), abs(nodes[ai].x - w.baseX) < 6,
                      let bi = nearestNode(seg: top, x: w.x + w.side * MANTLE_IN) else { continue }
                let s = segs[si]
                let jumpUp: CGFloat = s.y + HAND_REACH < w.y0 ? 0.3 : 0
                adj[ai].append(Edge(to: bi, cost: (w.y1 - s.y) / CLIMB_SPEED + 0.4 + jumpUp, kind: .climb(wi)))
            }
        }
    }

    var edgeCount: Int { adj.reduce(0) { $0 + $1.count } }

    /// What can be done about the cursor while standing at x on segment `seg`.
    private func evalGoal(seg: Int, x: CGFloat, _ c: CGPoint) -> (extra: CGFloat, residual: CGFloat, kind: GoalKind) {
        let s = segs[seg]
        let dx = c.x - x, h = c.y - s.y
        if h < -3 {
            // Cursor is below: drop through the platform and snatch it on the way down.
            if abs(dx) < 6 && (s.owner >= 0 || s.owner <= VISION_OWNER) {
                let handStop = c.y - HAND_REACH + 4
                let blocked = segsInY(handStop + 0.01, s.y - 1).contains { j in j != seg && segs[j].contains(x, 3) }
                if !blocked { return ((2 * max(s.y - handStop, 1) / GRAV).squareRoot() + 0.25, 0, .dropCatch) }
            }
            return (0, hypot(dx, h) + 30, .frustrated)
        }
        if h < 22 {
            if let j = solveJump(dx: dx, dy: max(h, 0)) { return (j.t + 0.1, 0, .catchJump) }
            return (0, abs(dx) + 20, .frustrated)
        }
        let need = h - HAND_REACH + 4
        if need <= 0 && abs(dx) < 10 { return (0.05, 0, .walkGrab) }
        if need <= JUMP_MAX {
            let apex = clampf(need + 4, 10, JUMP_MAX)
            let reach = (2 * apex / GRAV).squareRoot() * AIR_VX_MAX * 0.85
            if abs(dx) <= reach { return ((2 * apex / GRAV).squareRoot() + 0.1 + abs(dx) / RUN_SPEED * 0.5, 0, .catchJump) }
            return (0, abs(dx) - reach, .frustrated)
        }
        return (0, need - JUMP_MAX + max(0, abs(dx) - 30), .frustrated)
    }

    func plan(seg: Int, x: CGFloat, cursor c: CGPoint, prevSig: Int, climbMult: CGFloat) -> (Action, [CGPoint], Int)? {
        let n = nodes.count
        guard n > 0, seg < segNodes.count, !segNodes[seg].isEmpty else { return nil }
        var dist = [CGFloat](repeating: .infinity, count: n)
        var prev = [Int](repeating: -2, count: n)
        var prevEdge = [Int](repeating: -1, count: n)
        var firstSig = [Int](repeating: 0, count: n)
        var heap = MinHeap()
        var li = -1, ri = -1
        for id in segNodes[seg] { if nodes[id].x <= x { li = id } else { ri = id; break } }
        for id in [li, ri] where id >= 0 {
            let d = abs(nodes[id].x - x) / RUN_SPEED
            if d < dist[id] { dist[id] = d; prev[id] = -1; heap.push(d, id) }
        }
        while !heap.isEmpty {
            let (d, u) = heap.pop()
            if d > dist[u] { continue }
            for (k, e) in adj[u].enumerated() {
                var cost = e.cost
                var es = 0
                let dst = segs[nodes[e.to].seg]
                switch e.kind {
                case .walk: break
                case .jump: es = sig(1, Int(dst.y), Int(dst.x0))
                case .ledge: es = sig(2, Int(dst.y), Int(dst.x0))
                case .drop: es = sig(3, Int(dst.y), Int(dst.x0))
                case .climb(let wi): cost *= climbMult; es = sig(4, Int(walls[wi].x), Int(walls[wi].side))
                }
                let nd = d + cost
                if nd < dist[e.to] {
                    dist[e.to] = nd; prev[e.to] = u; prevEdge[e.to] = k
                    firstSig[e.to] = firstSig[u] != 0 ? firstSig[u] : es
                    heap.push(nd, e.to)
                }
            }
        }

        var bestScore = CGFloat.infinity
        var bestNode = -1
        var bestFrom: CGFloat = 0
        var bestKind = GoalKind.frustrated
        var bestWall: Wall? = nil
        var bestSig = 0
        for i in 0..<n where dist[i] < .infinity {
            let nd = nodes[i], s = segs[nd.seg]
            let cx = clampf(c.x, s.x0 + 3, s.x1 - 3)
            let from = abs(cx - nd.x) <= NODE_STRIDE * 0.5 + 2 ? cx : nd.x
            let g = evalGoal(seg: nd.seg, x: from, c)
            let sg = firstSig[i] != 0 ? firstSig[i] : sig(9, Int(s.y), g.residual > 0 ? 1 : 0)
            var score = dist[i] + abs(from - nd.x) / RUN_SPEED + g.extra + (g.residual > 0 ? 2 + g.residual * 0.03 : 0)
            if sg == prevSig { score -= 0.4 }
            if score < bestScore { bestScore = score; bestNode = i; bestFrom = from; bestKind = g.kind; bestWall = nil; bestSig = sg }
        }
        // Climb a wall and grab the cursor from it, or leap off the wall at it.
        for w in walls {
            let ddx = (c.x - w.x) * -w.side
            guard ddx > -12 && ddx < LEAP_RANGE && c.y <= w.y1 + 25 && c.y >= w.y0 + 6 else { continue }
            for si in segsInY(w.y0 - WALL_GRAB_REACH, min(w.y1 - 15, c.y - HAND_REACH - 10)) where climbBase(segs[si], w) {
                guard let ni = nearestNode(seg: si, x: w.baseX), dist[ni] < .infinity else { continue }
                let s = segs[si]
                let climbT = max(0, c.y - HAND_REACH - s.y) / CLIMB_SPEED * climbMult
                let sg = firstSig[ni] != 0 ? firstSig[ni] : sig(8, Int(w.x), Int(w.side))
                var score = dist[ni] + abs(nodes[ni].x - w.baseX) / RUN_SPEED + climbT + max(0, ddx) / 400 + 0.45
                if sg == prevSig { score -= 0.4 }
                if score < bestScore { bestScore = score; bestNode = ni; bestFrom = w.baseX; bestWall = w; bestSig = sg }
            }
        }
        guard bestNode >= 0 else { return nil }

        var chain: [Int] = []
        var cur = bestNode
        while cur >= 0 { chain.append(cur); cur = prev[cur] }
        chain.reverse()
        var pts = [P(x, segs[seg].y)]
        for id in chain { pts.append(P(nodes[id].x, segs[nodes[id].seg].y)) }

        if chain.count > 1 {
            for i in 1..<chain.count {
                let from = nodes[chain[i - 1]]
                let to = nodes[chain[i]]
                let e = adj[chain[i - 1]][prevEdge[chain[i]]]
                let toPt = P(to.x, segs[to.seg].y)
                switch e.kind {
                case .walk: continue
                case .jump: return (.jump(from: from.x, to: toPt, target: segs[to.seg], ledge: false), pts, bestSig)
                case .ledge: return (.jump(from: from.x, to: toPt, target: segs[to.seg], ledge: true), pts, bestSig)
                case .drop: return (.drop(from: from.x), pts, bestSig)
                case .climb(let wi): return (.climb(from: from.x, wall: walls[wi], leap: false), pts, bestSig)
                }
            }
        }
        if let w = bestWall {
            pts.append(P(w.x, c.y))
            return (.climb(from: bestFrom, wall: w, leap: true), pts, bestSig)
        }
        pts.append(c)
        switch bestKind {
        case .catchJump: return (.catchJump, pts, bestSig)
        case .walkGrab: return (.walk, pts, bestSig)
        case .dropCatch: return (.dropCatch(from: bestFrom), pts, bestSig)
        case .frustrated: return (.frustrated(at: bestFrom), pts, bestSig)
        }
    }
}

// MARK: - Edge detection

/// Finds horizontal and vertical edges in a luminance image (row 0 at the top) and returns them as
/// flat (x0, x1, y) and (y0, y1, x) runs in image pixels. Edges are traced pixel to pixel, allowing a
/// one-pixel wobble per step, so soft or gently curved outlines (rounded buttons, icons) still count;
/// a curve is split into short flat steps.
func detectEdges(_ lum: [UInt8], _ w: Int, _ h: Int) -> (h: [(Int, Int, Int)], v: [(Int, Int, Int)]) {
    guard w > 8 && h > 8 else { return ([], []) }
    let T: Int16 = 17
    var hs: [(Int, Int, Int, Int)] = []
    var vs: [(Int, Int, Int, Int)] = []
    var g = [Int16](repeating: 0, count: w * h)
    var mask = [Bool](repeating: false, count: w * h)
    var seen = [Bool](repeating: false, count: w * h)

    /// Traces chains through `mask`. `along` steps the main axis, `across` the cross axis.
    func trace(len nMain: Int, cross nCross: Int, index: (Int, Int) -> Int, minLen: Int, out: inout [(Int, Int, Int, Int)]) {
        for i in 0..<seen.count { seen[i] = false }
        for c0 in 2..<(nCross - 2) {
            for m0 in 0..<nMain where mask[index(m0, c0)] && !seen[index(m0, c0)] {
                var m = m0, c = c0, gap = 0
                var pieceStart = m0, pieceC = c0, hits = 1, sum = Int(g[index(m0, c0)])
                seen[index(m0, c0)] = true
                func close(_ end: Int) {
                    let len = end - pieceStart + 1
                    if len >= minLen && hits * 100 >= len * 93 { out.append((pieceStart, end, pieceC, sum)) }
                }
                while m + 1 < nMain {
                    var next = -1
                    for dc in [0, -1, 1] {
                        let cc = c + dc
                        if cc >= 1 && cc < nCross - 1 && mask[index(m + 1, cc)] && !seen[index(m + 1, cc)] { next = cc; break }
                    }
                    if next >= 0 {
                        m += 1; c = next; gap = 0
                        seen[index(m, c)] = true
                        if abs(c - pieceC) > 2 {
                            close(m - 1)
                            pieceStart = m; pieceC = c; hits = 0; sum = 0
                        }
                        hits += 1; sum += Int(g[index(m, c)])
                    } else if gap < 1 {
                        m += 1; gap += 1
                    } else {
                        break
                    }
                }
                close(m - gap)
            }
        }
    }

    lum.withUnsafeBufferPointer { L in
        // Horizontal edges: brightness change from the row above to the row below, thinned to the peak row.
        for y in 1..<(h - 1) {
            let up = (y - 1) * w, dn = (y + 1) * w, me = y * w
            for x in 0..<w { g[me + x] = Int16(abs(Int(L[dn + x]) - Int(L[up + x]))) }
        }
        for y in 2..<(h - 2) {
            let me = y * w
            for x in 0..<w { let v = g[me + x]; mask[me + x] = v >= T && v >= g[me - w + x] && v > g[me + w + x] }
        }
    }
    trace(len: w, cross: h, index: { m, c in c * w + m }, minLen: 22, out: &hs)

    lum.withUnsafeBufferPointer { L in
        // Vertical edges: brightness change from the column left to the column right.
        for y in 0..<h {
            let me = y * w
            g[me] = 0; g[me + w - 1] = 0
            for x in 1..<(w - 1) { g[me + x] = Int16(abs(Int(L[me + x + 1]) - Int(L[me + x - 1]))) }
        }
        for y in 0..<h {
            let me = y * w
            mask[me] = false; mask[me + 1] = false; mask[me + w - 1] = false; mask[me + w - 2] = false
            for x in 2..<(w - 2) { let v = g[me + x]; mask[me + x] = v >= T && v >= g[me + x - 1] && v > g[me + x + 1] }
        }
    }
    trace(len: h, cross: w, index: { m, c in m * w + c }, minLen: 26, out: &vs)

    // Strongest first; drop near-duplicates (a 1px line gives an edge on each side).
    func dedupe(_ runs: [(Int, Int, Int, Int)], cap: Int) -> [(Int, Int, Int)] {
        var kept: [(Int, Int, Int)] = []
        for r in runs.sorted(by: { $0.3 > $1.3 }).prefix(cap * 3) {
            let dup = kept.contains { k in
                abs(k.2 - r.2) <= 3 && min(k.1, r.1) - max(k.0, r.0) > (min(k.1 - k.0, r.1 - r.0)) / 2
            }
            if !dup { kept.append((r.0, r.1, r.2)) }
            if kept.count >= cap { break }
        }
        return kept
    }
    return (dedupe(hs, cap: 320), dedupe(vs, cap: 140))
}

/// Converts detected runs into world surfaces for a screen whose Cocoa frame is `frame`.
func surfaces(from e: (h: [(Int, Int, Int)], v: [(Int, Int, Int)]), imgW: Int, imgH: Int, frame: CGRect, lum: [UInt8]) -> ([Seg], [Wall]) {
    let sx = frame.width / CGFloat(imgW), sy = frame.height / CGFloat(imgH)
    var segs: [Seg] = []
    for (i, r) in e.h.enumerated() {
        segs.append(Seg(x0: frame.minX + CGFloat(r.0) * sx, x1: frame.minX + CGFloat(r.1 + 1) * sx,
                        y: frame.maxY - CGFloat(r.2) * sy, owner: VISION_OWNER - i))
    }
    var walls: [Wall] = []
    for (i, r) in e.v.enumerated() {
        // The darker/busier side is treated as the solid side he climbs against: pick the side
        // whose brightness differs more from the screen's typical background is unknowable, so
        // alternate by comparing a few pixels either side and put him on the brighter side.
        let x = r.2, ym = (r.0 + r.1) / 2
        let l = Int(lum[ym * imgW + max(x - 3, 0)]), rr = Int(lum[ym * imgW + min(x + 3, imgW - 1)])
        let side: CGFloat = l > rr ? 1 : -1
        walls.append(Wall(x: frame.minX + CGFloat(x) * sx, y0: frame.maxY - CGFloat(r.1 + 1) * sy, y1: frame.maxY - CGFloat(r.0) * sy,
                          side: side, owner: VISION_OWNER - 5000 - i, hasTop: false))
    }
    return (segs, walls)
}

/// Keeps only edges that also appeared in the previous frame (or are long), so video and
/// animation noise doesn't create flickering ledges.
func stableRuns(_ now: [(Int, Int, Int)], _ before: [(Int, Int, Int)], longEnough: Int) -> [(Int, Int, Int)] {
    now.filter { r in
        r.1 - r.0 >= longEnough || before.contains { b in
            abs(b.2 - r.2) <= 2 && min(b.1, r.1) - max(b.0, r.0) >= (r.1 - r.0) * 6 / 10
        }
    }
}

final class Vision: NSObject, SCStreamOutput, SCStreamDelegate {
    private var streams: [SCStream] = []
    private var displayFor: [ObjectIdentifier: CGDirectDisplayID] = [:]
    private var frames: [CGDirectDisplayID: CGRect] = [:]
    private var previous: [CGDirectDisplayID: (h: [(Int, Int, Int)], v: [(Int, Int, Int)])] = [:]
    private var results: [CGDirectDisplayID: ([Seg], [Wall])] = [:]
    private let queue = DispatchQueue(label: "stickchase.vision", qos: .utility)
    var onUpdate: (([Seg], [Wall]) -> Void)?
    private(set) var running = false

    func start(completion: @escaping (Bool) -> Void) {
        var map: [CGDirectDisplayID: CGRect] = [:]
        for s in NSScreen.screens {
            if let n = s.deviceDescription[NSDeviceDescriptionKey("NSScreenNumber")] as? NSNumber { map[n.uint32Value] = s.frame }
        }
        frames = map
        SCShareableContent.getExcludingDesktopWindows(false, onScreenWindowsOnly: true) { [weak self] content, _ in
            guard let self, let content else { DispatchQueue.main.async { completion(false) }; return }
            let me = content.applications.filter { $0.processID == getpid() }
            var started = 0
            for d in content.displays {
                guard self.frames[d.displayID] != nil else { continue }
                let filter = SCContentFilter(display: d, excludingApplications: me, exceptingWindows: [])
                let cfg = SCStreamConfiguration()
                cfg.width = d.width
                cfg.height = d.height
                cfg.minimumFrameInterval = CMTime(value: 1, timescale: 4)
                cfg.pixelFormat = kCVPixelFormatType_32BGRA
                cfg.showsCursor = false
                cfg.queueDepth = 3
                let stream = SCStream(filter: filter, configuration: cfg, delegate: self)
                do {
                    try stream.addStreamOutput(self, type: .screen, sampleHandlerQueue: self.queue)
                    self.displayFor[ObjectIdentifier(stream)] = d.displayID
                    self.streams.append(stream)
                    stream.startCapture { _ in }
                    started += 1
                } catch {}
            }
            DispatchQueue.main.async { self.running = started > 0; completion(started > 0) }
        }
    }

    func stop() {
        for s in streams { s.stopCapture { _ in } }
        streams = []
        running = false
    }

    func stream(_ stream: SCStream, didStopWithError error: Error) {
        DispatchQueue.main.async { self.running = false }
    }

    func stream(_ stream: SCStream, didOutputSampleBuffer sb: CMSampleBuffer, of type: SCStreamOutputType) {
        guard type == .screen, let pb = sb.imageBuffer,
              let id = displayFor[ObjectIdentifier(stream)], let frame = frames[id] else { return }
        if let att = CMSampleBufferGetSampleAttachmentsArray(sb, createIfNecessary: false) as? [[SCStreamFrameInfo: Any]],
           let raw = att.first?[.status] as? Int, let status = SCFrameStatus(rawValue: raw), status != .complete { return }
        CVPixelBufferLockBaseAddress(pb, .readOnly)
        let w = CVPixelBufferGetWidth(pb), h = CVPixelBufferGetHeight(pb), bpr = CVPixelBufferGetBytesPerRow(pb)
        guard let base = CVPixelBufferGetBaseAddress(pb)?.assumingMemoryBound(to: UInt8.self) else {
            CVPixelBufferUnlockBaseAddress(pb, .readOnly); return
        }
        var lum = [UInt8](repeating: 0, count: w * h)
        lum.withUnsafeMutableBufferPointer { L in
            for y in 0..<h {
                let row = base + y * bpr
                for x in 0..<w {
                    let p = row + x * 4
                    let r: Int = Int(p[2]) * 77, g: Int = Int(p[1]) * 150, b: Int = Int(p[0]) * 29
                    L[y * w + x] = UInt8((r + g + b) >> 8)
                }
            }
        }
        CVPixelBufferUnlockBaseAddress(pb, .readOnly)
        let found = detectEdges(lum, w, h)
        let prev = previous[id] ?? found
        let stable = (h: stableRuns(found.h, prev.h, longEnough: 80), v: stableRuns(found.v, prev.v, longEnough: 90))
        previous[id] = found
        results[id] = surfaces(from: stable, imgW: w, imgH: h, frame: frame, lum: lum)
        var allSegs: [Seg] = [], allWalls: [Wall] = []
        for (_, r) in results { allSegs += r.0; allWalls += r.1 }
        // Owners must be unique across displays.
        for i in allSegs.indices { allSegs[i].owner = VISION_OWNER - i }
        for i in allWalls.indices { allWalls[i].owner = VISION_OWNER - 5000 - i }
        DispatchQueue.main.async { self.onUpdate?(allSegs, allWalls) }
    }
}

// MARK: - Poses

/// Local coordinates in body units: origin between the feet, +x is the way he faces, +y is up.
struct Pose {
    var hip: CGPoint, neck: CGPoint, head: CGPoint
    var hL: CGPoint, hR: CGPoint, fL: CGPoint, fR: CGPoint
    var pivot = P(0, 28)
    var rot: CGFloat = 0
    var sx: CGFloat = 1          // horizontal squash, used for twists
    var shoulder: CGPoint { lerp(neck, hip, 0.1) }
    var joints: [CGPoint] {
        get { [hip, neck, head, hL, hR, fL, fR] }
        set { hip = newValue[0]; neck = newValue[1]; head = newValue[2]; hL = newValue[3]; hR = newValue[4]; fL = newValue[5]; fR = newValue[6] }
    }

    func blended(to b: Pose, _ k: CGFloat) -> Pose {
        Pose(hip: lerp(hip, b.hip, k), neck: lerp(neck, b.neck, k), head: lerp(head, b.head, k),
             hL: lerp(hL, b.hL, k), hR: lerp(hR, b.hR, k), fL: lerp(fL, b.fL, k), fR: lerp(fR, b.fR, k),
             pivot: lerp(pivot, b.pivot, k), rot: rot + angleDiff(rot, b.rot) * k, sx: lerp(sx, b.sx, k))
    }
    func shifted(_ d: CGPoint) -> Pose {
        var p = self
        p.joints = joints.map { $0 + d }
        p.pivot = pivot + d
        return p
    }
    /// The same pose with left and right limbs swapped (for the second half of a cycle).
    var mirrored: Pose {
        var p = self
        swap(&p.hL, &p.hR); swap(&p.fL, &p.fR)
        return p
    }
}

func mix(_ a: Pose, _ b: Pose, _ k: CGFloat) -> Pose { a.blended(to: b, clampf(k, 0, 1)) }

/// Smoothly interpolates a looping set of key poses (times in 0..<1) with Catmull-Rom splines.
func cyclePose(_ keys: [(CGFloat, Pose)], _ phase: CGFloat) -> Pose {
    let t = frac(phase)
    let n = keys.count
    var i = n - 1
    for k in 0..<n where keys[k].0 <= t { i = k }
    let j = (i + 1) % n
    let t0 = keys[i].0, t1 = j == 0 ? 1 + keys[0].0 : keys[j].0
    let u = clampf((t - t0 + (t < t0 ? 1 : 0)) / max(t1 - t0, 1e-4), 0, 1)
    let a = keys[(i + n - 1) % n].1, b = keys[i].1, c = keys[j].1, d = keys[(j + 1) % n].1
    var out = b
    let ja = a.joints, jb = b.joints, jc = c.joints, jd = d.joints
    out.joints = (0..<7).map { catmull(ja[$0], jb[$0], jc[$0], jd[$0], u) }
    return out
}

enum Poses {
    /// Builds a pose from hip position and torso lean (radians, + is forward). Hands are relative to the shoulder.
    static func make(_ hip: CGPoint, _ lean: CGFloat, tilt: CGFloat = 0, hl: CGPoint, hr: CGPoint,
                     fl: CGPoint, fr: CGPoint, pivot: CGPoint = P(0, 28)) -> Pose {
        let neck = hip + P(18 * sin(lean), 18 * cos(lean))
        let head = neck + P(8.3 * sin(lean + tilt), 8.3 * cos(lean + tilt))
        let s = lerp(neck, hip, 0.1)
        return Pose(hip: hip, neck: neck, head: head, hL: s + hl, hR: s + hr, fL: fl, fR: fr, pivot: pivot)
    }

    static func stand(_ t: CGFloat, look: CGFloat = 0) -> Pose {
        let b = sin(t * 2.1)
        let shift = sin(t * 0.45)          // slow weight shift from foot to foot
        return make(P(0.8 * shift, 25.1 + 0.25 * b), 0.03 + 0.012 * b - 0.02 * shift, tilt: look,
                    hl: P(-3.5, -19.5 + 0.3 * b), hr: P(3.5, -19.6 + 0.3 * b), fl: P(-4, 0), fr: P(4.5, 0))
    }

    // Run: contact -> down -> push -> peak for the near (right) leg, mirrored for the far leg.
    // The stance foot slides back 19 units between contact (0) and push (0.18), so one full cycle
    // covers 19 * S / 0.18 points of ground.
    static let runKeys: [(CGFloat, Pose)] = {
        let contact = make(P(0, 23.8), 0.2, tilt: -0.08, hl: P(7, -7), hr: P(-8, -12), fl: P(-12, 6), fr: P(11, 0.4))
        let down = make(P(-0.5, 21.2), 0.26, tilt: -0.1, hl: P(5, -9), hr: P(-5, -13), fl: P(-8, 12), fr: P(3.6, 0))
        let push = make(P(0.5, 24.2), 0.28, tilt: -0.1, hl: P(-4, -13), hr: P(8, -6), fl: P(7, 12), fr: P(-8, 0.3))
        let peak = make(P(0, 26.2), 0.22, tilt: -0.08, hl: P(-8, -11), hr: P(9, -5), fl: P(10, 4), fr: P(-12, 7))
        return [(0, contact), (0.07, down), (0.18, push), (0.34, peak),
                (0.5, contact.mirrored), (0.57, down.mirrored), (0.68, push.mirrored), (0.84, peak.mirrored)]
    }()
    static let runCycleLen: CGFloat = 19 * S / 0.18

    // Walk: contact -> down -> passing -> up; stance foot travels 20 units over half a cycle.
    static let walkKeys: [(CGFloat, Pose)] = {
        let contact = make(P(0, 24.2), 0.04, hl: P(5, -17.5), hr: P(-4, -18.5), fl: P(-10, 1), fr: P(10, 0.3))
        let down = make(P(0, 23.4), 0.05, hl: P(3.5, -18), hr: P(-3, -18.8), fl: P(-9.5, 3), fr: P(6.8, 0))
        let passing = make(P(0, 25.2), 0.04, hl: P(0.5, -19), hr: P(0.5, -19), fl: P(1, 4.5), fr: P(0, 0))
        let up = make(P(0, 25.7), 0.03, hl: P(-2.5, -18.8), hr: P(3, -18.5), fl: P(7, 1.8), fr: P(-5, 0))
        return [(0, contact), (0.08, down), (0.25, passing), (0.38, up),
                (0.5, contact.mirrored), (0.58, down.mirrored), (0.75, passing.mirrored), (0.88, up.mirrored)]
    }()
    static let walkCycleLen: CGFloat = 20 * S / 0.5

    /// Blend weight between walk (0) and run (1) for a ground speed.
    static func gait(_ speed: CGFloat) -> CGFloat { clampf((speed - 60) / 100, 0, 1) }
    static func cycleLength(_ speed: CGFloat) -> CGFloat { lerp(walkCycleLen, runCycleLen, gait(speed)) }

    static func locomote(_ ph: CGFloat, _ speed: CGFloat) -> Pose {
        let w = gait(speed)
        var p = w <= 0 ? cyclePose(walkKeys, ph) : (w >= 1 ? cyclePose(runKeys, ph) : mix(cyclePose(walkKeys, ph), cyclePose(runKeys, ph), w))
        if speed > RUN_SPEED * 1.1 {
            // Sprinting: more lean and bigger arm swing.
            let k = clampf((speed - RUN_SPEED * 1.1) / 60, 0, 1)
            p.neck = p.neck + P(2 * k, -0.5 * k); p.head = p.head + P(3 * k, -1 * k)
            let s = p.shoulder
            p.hL = s + (p.hL - s) * (1 + 0.25 * k); p.hR = s + (p.hR - s) * (1 + 0.25 * k)
        }
        p.fL.y = max(p.fL.y, 0); p.fR.y = max(p.fR.y, 0)
        return p
    }

    static let skid = make(P(-4, 21.5), -0.35, tilt: 0.2, hl: P(8, -4), hr: P(11, -1), fl: P(9, 0), fr: P(1, 0))
    static let windup = make(P(-2, 16), 0.5, tilt: -0.35, hl: P(-10, -9), hr: P(-8, -11), fl: P(-5, 0), fr: P(6, 0))

    // Jump poses keyed by vertical speed: stretch on take-off, tuck at the top, reach for the landing.
    static let jStretch = make(P(0, 27.5), 0.16, tilt: -0.15, hl: P(5, 19), hr: P(10, 15), fl: P(-7, 1.5), fr: P(7, 13))
    static let jRise = make(P(0, 27), 0.14, tilt: -0.1, hl: P(-4, 13), hr: P(11, 11), fl: P(-6, 7), fr: P(6, 13))
    static let jApex = make(P(0, 26.5), 0.12, hl: P(-11, 5), hr: P(12, 6), fl: P(-1, 11), fr: P(6, 12))
    static let jFall = make(P(0, 26), -0.04, tilt: 0.3, hl: P(-12, 9), hr: P(12, 10), fl: P(-5, 4), fr: P(5, 2))
    static let jFallFast = make(P(0, 25.5), -0.06, tilt: 0.35, hl: P(-11, 14), hr: P(12, 15), fl: P(-4, 1), fr: P(4, 0))
    /// k: +1 rising fast, 0 apex, -1 falling fast.
    static func jump(_ k: CGFloat, _ t: CGFloat) -> Pose {
        var p: Pose
        switch k {
        case 0.45...: p = mix(jRise, jStretch, (k - 0.45) / 0.55)
        case 0..<0.45: p = mix(jApex, jRise, k / 0.45)
        case -0.5..<0: p = mix(jApex, jFall, -k / 0.5)
        default: p = mix(jFall, jFallFast, (-k - 0.5) / 0.5)
        }
        let flail = max(0, -k - 0.7) * 3
        p.hL = p.hL + P(sin(t * 17), cos(t * 13)) * (2.5 * flail)
        p.hR = p.hR + P(cos(t * 15), sin(t * 19)) * (2.5 * flail)
        return p
    }
    static let tuck = make(P(0, 24), 0.35, hl: P(6.8, -18.8), hr: P(7.8, -20), fl: P(7.5, 15), fr: P(10, 13), pivot: P(3, 30))
    static let roll = tuck.shifted(P(0, -12))
    static let crouchDeep = make(P(-3, 13.5), 0.5, tilt: -0.35, hl: P(9, -12), hr: P(12, -9), fl: P(-6, 0), fr: P(7, 0))
    static func crouch(_ d: CGFloat) -> Pose { mix(stand(0), crouchDeep, d) }
    static let hero = make(P(-2, 11), 0.5, tilt: -0.5, hl: P(-12, 6), hr: P(6.2, -24.7), fl: P(-11, 0), fr: P(7, 0))
    static func stumble(_ t: CGFloat) -> Pose {
        make(P(1, 22), 0.55, tilt: -0.4, hl: P(10 + 4 * sin(t * 20), 4 + 4 * cos(t * 20)), hr: P(12 + 4 * cos(t * 20), 2 + 4 * sin(t * 20)),
             fl: P(10, 1), fr: P(-6, 4))
    }
    /// Hand-over-hand climbing. A gripping hand stays fixed on the wall (it slides down in body space
    /// as the body rises) while the other reaches up; one full cycle raises the body 28 units.
    static func climb(_ ph: CGFloat) -> Pose {
        func hand(_ p: CGFloat) -> CGFloat {
            let u = frac(p)
            return u < 0.5 ? 60 - 28 * u : 46 + 14 * ease((u - 0.5) / 0.5)
        }
        func foot(_ p: CGFloat) -> CGFloat {
            let u = frac(p)
            return u < 0.5 ? 13 - 28 * u : -1 + 14 * ease((u - 0.5) / 0.5)
        }
        let sway = sin(ph * 4 * .pi)
        return Pose(hip: P(-1.5 + 1.2 * sway, 21), neck: P(1.5, 39), head: P(-0.5, 47.5 + 0.8 * sway),
                    hL: P(9.5, hand(ph)), hR: P(9.5, hand(ph + 0.5)), fL: P(7.5, foot(ph + 0.5)), fR: P(7.5, foot(ph)))
    }
    static func wallRun(_ ph: CGFloat) -> Pose {
        func foot(_ p: CGFloat) -> CGFloat {
            let u = frac(p)
            return u < 0.5 ? 12 - 24 * u : 0 + 12 * ease((u - 0.5) / 0.5)
        }
        let s1 = sin(ph * 2 * .pi)
        return Pose(hip: P(0, 23), neck: P(-1.5, 41), head: P(-1.5, 49.5),
                    hL: P(3 - 5 * s1, 29), hR: P(3 + 5 * s1, 29), fL: P(9, foot(ph)), fR: P(9, foot(ph + 0.5)))
    }
    static func ledgeHang(_ swing: CGFloat) -> Pose {
        Pose(hip: P(swing * 2, 21.5), neck: P(1.5, 40.5), head: P(-3.5, 46),
             hL: P(6.5, 60), hR: P(7.5, 60), fL: P(-1 + swing * 5, 0), fR: P(2 + swing * 5, 1))
    }
    static func mantle(_ u: CGFloat, _ hl: CGPoint) -> Pose {
        if u < 0.5 {
            let e = ease(u / 0.5)
            return Pose(hip: P(-1, 21 + 3 * e), neck: P(2, 39 + 2 * e), head: P(0, 47 + 2 * e),
                        hL: hl, hR: hl + P(1, 0), fL: P(5, 5 + 13 * e), fR: P(6, 3 + 8 * e))
        }
        let e = ease((u - 0.5) / 0.5)
        let hip = P(-3 + 3 * e, 14 + 11 * e), neck = P(5 - 3.8 * e, 30 + 13.3 * e)
        let s = lerp(neck, hip, 0.1)
        return Pose(hip: hip, neck: neck, head: P(9 - 6.6 * e, 37.5 + 14.3 * e),
                    hL: lerp(hl, s + P(-3.5, -19.5), e), hR: lerp(hl + P(1, 0), s + P(3.5, -19.5), e),
                    fL: P(-5 + e, 0), fR: P(6 - 1.5 * e, 0))
    }
    static func hang(_ lag: CGFloat, _ pull: CGFloat, _ kick: CGFloat) -> Pose {
        Pose(hip: P(lag * 0.5, 24.5 + 14 * pull), neck: P(0, 42.5 + 14 * pull), head: P(-5, 47.5 + 13 * pull),
             hL: P(-0.5, 62), hR: P(1.5, 61.5),
             fL: P(lag - 2 + 3 * kick, 1.5 + 10 * pull), fR: P(lag + 3 - 3 * kick, 0.5 + 8 * pull), pivot: P(0, 62))
    }
    static func ride(_ t: CGFloat) -> Pose {
        var p = make(P(0, 25), 0, hl: P(-16, 3 + 5 * sin(t * 5)), hr: P(16, 3 - 5 * sin(t * 5)),
                     fl: P(-2.5, 0), fr: P(2.5, 0), pivot: P(0, 0))
        p.rot = 0.1 * sin(t * 3.3)
        return p
    }
    /// Reaching both hands toward a point in local body units.
    static func reach(_ target: CGPoint) -> Pose {
        let lean = clampf(atan2(target.x, max(target.y - 38, 4)) * 0.5, -0.35, 0.6)
        var p = make(P(0, 25.5), lean, tilt: -0.3, hl: .zero, hr: .zero, fl: P(-7, 6), fr: P(-2, 1.5))
        p.hL = target + P(-1, -1); p.hR = target
        return p
    }
    static func hold(_ t: CGFloat, _ target: CGPoint, _ ph: CGFloat, _ speed: CGFloat) -> Pose {
        var p = speed > 20 ? locomote(ph, speed) : stand(t, look: -0.3)
        p.hR = target
        p.hL = p.shoulder + P(-4, -18)
        return p
    }

    // Idle moods
    static func fist(_ t: CGFloat) -> Pose {
        var p = stand(t, look: -0.35)
        p.hR = p.shoulder + P(10, 15 + 3 * sin(t * 26)); p.hL = p.shoulder + P(-5, -14)
        return p
    }
    static func sit(_ t: CGFloat) -> Pose {
        var p = make(P(-2, 5.5), -0.2, tilt: 0.12 * sin(t * 0.8), hl: .zero, hr: .zero, fl: P(13, 0), fr: P(10, 3))
        p.hL = P(-11, 0.3); p.hR = P(-9, 0.5)
        return p
    }
    static func cross(_ t: CGFloat) -> Pose {
        var p = stand(t, look: -0.15)
        p.hL = p.shoulder + P(5, -7); p.hR = p.shoulder + P(4, -8.5)
        p.fR = P(5.5, max(0, sin(t * 12)) * 2.2)
        return p
    }
    static func lookUp(_ t: CGFloat) -> Pose {
        var p = stand(t, look: -0.6)
        p.hR = p.shoulder + P(7, 9); p.hL = p.shoulder + P(-3.5, -19)
        return p
    }
    static func wave(_ t: CGFloat) -> Pose {
        var p = stand(t, look: -0.2)
        p.hR = p.shoulder + P(9 + 3 * sin(t * 9), 16)
        return p
    }
    static func pushup(_ t: CGFloat) -> Pose {
        let u = (sin(t * 5) + 1) / 2
        return Pose(hip: P(-5, 5 + 3 * u), neck: P(12, 8 + 7 * u), head: P(19.5, 10 + 7.5 * u),
                    hL: P(10.5, 0), hR: P(11.5, 0), fL: P(-29, 0), fR: P(-28, 0.5))
    }
    static func stretch(_ t: CGFloat) -> Pose {
        make(P(0, 25.2), 0.2 * sin(t * 1.5), tilt: -0.2, hl: P(-2, 20), hr: P(2, 20), fl: P(-4, 0), fr: P(4.5, 0))
    }
    static func dance(_ t: CGFloat) -> Pose {
        let s6 = sin(t * 6)
        return make(P(2 * s6, 24 + abs(s6) * 1.5), 0.1 * sin(t * 3), tilt: 0.2 * sin(t * 3),
                    hl: P(-8, 10 * s6), hr: P(8, -10 * s6),
                    fl: P(-5 + s6, max(0, s6) * 3), fr: P(5 + s6, max(0, -s6) * 3))
    }
}

// MARK: - Springy rig (follow-through and overlapping action)

/// Second-order dynamics: each joint chases its keyed position like a damped spring, so heads and
/// hands lag, overshoot and settle instead of snapping between poses.
struct JointSpring {
    var y: CGPoint, v = CGPoint.zero
    let f: CGFloat, z: CGFloat
    mutating func update(_ x: CGPoint, _ dt: CGFloat, _ boost: CGFloat) -> CGPoint {
        let ff = f * boost
        let k1 = z / (.pi * ff), k2 = 1 / ((2 * .pi * ff) * (2 * .pi * ff))
        let k2s = max(k2, dt * dt / 2 + dt * k1 / 2, dt * k1)
        y = y + v * dt
        v = v + (x - y - v * k1) * (dt / k2s)
        return y
    }
}

struct Rig {
    // hip, neck, head, hand L, hand R, foot L, foot R
    static let params: [(CGFloat, CGFloat)] = [(8, 0.85), (7, 0.7), (5, 0.45), (6, 0.55), (6, 0.55), (15, 0.95), (15, 0.95)]
    static let dragWeight: [CGFloat] = [0, 0.35, 0.8, 1, 1, 0.15, 0.15]
    var joints: [JointSpring]
    var rot: CGFloat = 0, rotV: CGFloat = 0
    var pivot: CGPoint, sx: CGFloat = 1

    init(_ p: Pose) {
        let js = p.joints
        joints = (0..<7).map { JointSpring(y: js[$0], f: Rig.params[$0].0, z: Rig.params[$0].1) }
        pivot = p.pivot
        rot = p.rot
    }

    /// drag: inertia offset (body units, local) that pulls loose parts against the body's acceleration.
    mutating func update(_ target: Pose, _ dt: CGFloat, boost: CGFloat, drag: CGPoint) -> Pose {
        var out = target
        let tj = target.joints
        out.joints = (0..<7).map { joints[$0].update(tj[$0] + drag * Rig.dragWeight[$0], dt, boost) }
        // Rotation spring on the shortest angle.
        let x = rot + angleDiff(rot, target.rot)
        let ff = 7 * boost, z: CGFloat = 0.8
        let k1 = z / (.pi * ff), k2 = 1 / ((2 * .pi * ff) * (2 * .pi * ff))
        let k2s = max(k2, dt * dt / 2 + dt * k1 / 2, dt * k1)
        rot += rotV * dt
        rotV += (x - rot - rotV * k1) * (dt / k2s)
        out.rot = rot
        pivot = lerp(pivot, target.pivot, min(1, 20 * dt))
        sx = lerp(sx, target.sx, min(1, 25 * dt))
        out.pivot = pivot; out.sx = sx
        return out
    }
}

// MARK: - Drawing

func ik(_ a: CGPoint, _ t: CGPoint, _ l1: CGFloat, _ l2: CGFloat, _ bend: CGFloat) -> (CGPoint, CGPoint) {
    let dx = t.x - a.x, dy = t.y - a.y
    let d0 = hypot(dx, dy)
    let phi = d0 < 0.001 ? -CGFloat.pi / 2 : atan2(dy, dx)
    let d = clampf(d0, abs(l1 - l2) + 0.5, l1 + l2 - 0.01)
    let al = acos(clampf((l1 * l1 + d * d - l2 * l2) / (2 * l1 * d), -1, 1))
    return (P(a.x + l1 * cos(phi + bend * al), a.y + l1 * sin(phi + bend * al)),
            P(a.x + d * cos(phi), a.y + d * sin(phi)))
}

/// Near-side limbs + torso, far-side limbs, head: in screen coordinates.
func figurePaths(_ p: Pose, _ root: CGPoint, _ facing: CGFloat) -> (CGPath, CGPath, CGPath) {
    let cr = cos(p.rot), sr = sin(p.rot)
    func tf(_ q: CGPoint) -> CGPoint {
        let d = q - p.pivot
        let rx = d.x * cr - d.y * sr + p.pivot.x, ry = d.x * sr + d.y * cr + p.pivot.y
        return P(root.x + facing * S * p.sx * rx, root.y + S * ry)
    }
    let sh = p.shoulder
    let (kL, fL) = ik(p.hip, p.fL, LEG_U, LEG_L, 1)
    let (kR, fR) = ik(p.hip, p.fR, LEG_U, LEG_L, 1)
    let (eL, hL) = ik(sh, p.hL, ARM_U, ARM_L, -1)
    let (eR, hR) = ik(sh, p.hR, ARM_U, ARM_L, -1)
    let near = CGMutablePath(), far = CGMutablePath(), head = CGMutablePath()
    far.addLines(between: [p.hip, kL, fL].map(tf))
    far.addLines(between: [sh, eL, hL].map(tf))
    near.addLines(between: [p.hip, p.neck, lerp(p.neck, p.head, 0.35)].map(tf))
    near.addLines(between: [p.hip, kR, fR].map(tf))
    near.addLines(between: [sh, eR, hR].map(tf))
    let hc = tf(p.head), r = HEAD_R * S
    head.addEllipse(in: CGRect(x: hc.x - r, y: hc.y - r, width: 2 * r, height: 2 * r))
    return (near, far, head)
}

// MARK: - Cursor

struct CursorState {
    var p = CGPoint.zero, v = CGPoint.zero, a = CGPoint.zero
    var speed: CGFloat { v.len }
}

final class CursorTracker {
    var s = CursorState()
    private var prev: CGPoint?
    func update(_ p: CGPoint, _ dt: CGFloat) {
        guard let pp = prev, dt > 0 else { prev = p; s.p = p; return }
        let nv = lerp(s.v, (p - pp) * (1 / dt), 0.3)
        var acc = lerp(s.a, (nv - s.v) * (1 / dt), 0.3)
        if acc.len > 25000 { acc = acc * (25000 / acc.len) }
        s.a = acc; s.v = nv; s.p = p; prev = p
    }
}

// MARK: - Runner

enum Mode { case ground, air, wallRun, climb, ledge, mantle, hang, holdStand, ride }
enum Emote { case none, fist, sit, cross, lookUp, wave, pushup, stretch, dance, pace }
enum Trick { case none, front, back, twist }
enum Intent { case none, grab, hop }

struct Personality {
    var speed: CGFloat, flair: CGFloat, patience: CGFloat, climbPref: CGFloat
    static func random() -> Personality {
        Personality(speed: rnd(0.92...1.12), flair: rnd(0.3...0.8), patience: rnd(0.7...1.3), climbPref: rnd(0.8...1.2))
    }
}

struct Plan { var action: Action; var points: [CGPoint]; var made: CGFloat; var cursor: CGPoint; var version: Int }

struct FigureLayers { var near: CAShapeLayer; var far: CAShapeLayer; var head: CAShapeLayer; var ghosts: [CAShapeLayer] }

final class Runner {
    unowned let world: World
    let pers = Personality.random()
    var layers: [FigureLayers] = []

    var pos = CGPoint.zero, vel = CGPoint.zero
    var facing: CGFloat = 1
    var mode = Mode.air
    // ground
    var gOwner = 0
    var gRect: CGRect?
    var landT: CGFloat = 0, landDepth: CGFloat = 0.5, rollT: CGFloat = 0, heroT: CGFloat = 0, stumbleT: CGFloat = 0, preDropT: CGFloat = 0
    var dropCatching = false, skidding = false
    var emote = Emote.none, emoteT: CGFloat = 0, paceDir: CGFloat = 1
    var edgeHop = false
    var antT: CGFloat = 0
    var pending: (vx: CGFloat, vy: CGFloat, target: CGPoint?, intent: Intent, styled: Bool)?
    // air
    var airT: CGFloat = 0, peakY: CGFloat = 0, launchY: CGFloat = 0
    var ignoreAboveY: CGFloat?
    var jumpTarget: CGPoint?
    var wallIntent: Wall?
    var intent = Intent.none
    var trick = Trick.none, trickDur: CGFloat = 0.5
    // wall
    var wall: Wall?
    var wallRect: CGRect?
    var wallVy: CGFloat = 0, climbPhase: CGFloat = 0, climbStall: CGFloat = 0
    var leapIntent = false, dyno = false
    // ledge and mantle
    var edge = CGPoint.zero, edgeOwner = 0, ledgeT: CGFloat = 0
    var edgeRect: CGRect?
    var mantleT: CGFloat = 0, mantleDur: CGFloat = 0.32, mantleFrom = CGPoint.zero, mantleTo = CGPoint.zero
    // holding the cursor
    var theta: CGFloat = 0, omega: CGFloat = 0, grabCooldown: CGFloat = 0
    var spins: CGFloat = 0, grip: CGFloat = 0, dizzy = false
    var stillT: CGFloat = 0, trickT: CGFloat = 2, pullT: CGFloat = 0, kickT: CGFloat = 0, rideT: CGFloat = 0, hopT: CGFloat = 0
    // planning
    var plan: Plan?
    var lastSig = 0
    var styleClimb: CGFloat = 1, styleT: CGFloat = 0
    // animation
    var t: CGFloat = 0, runPhase: CGFloat = 0
    var rig = Rig(Poses.jump(-1, 0))
    var displayPose = Poses.jump(-1, 0)
    var drawOffset = CGPoint.zero
    var lastPos = CGPoint.zero, lastVel = CGPoint.zero, accel = CGPoint.zero
    var ghostPaths: [CGPath] = []
    var smear: CGFloat = 0
    var lastRoot = CGPoint.zero, lastRot: CGFloat = 0

    init(world: World, cursor: CGPoint) {
        self.world = world
        respawn(cursor)
        lastPos = pos
    }

    // MARK: Frame

    func step(_ dt: CGFloat, _ cur: CursorState) {
        t += dt
        grabCooldown -= dt
        styleT -= dt
        if styleT <= 0 { styleT = rnd(5...9); styleClimb = pers.climbPref * rnd(0.85...1.2) }
        let prev = pos
        switch mode {
        case .ground: stepGround(dt, cur)
        case .air: stepAir(dt, cur)
        case .wallRun: stepWallRun(dt, cur)
        case .climb: stepClimb(dt, cur)
        case .ledge: stepLedge(dt)
        case .mantle: stepMantle(dt)
        case .hang: stepHang(dt, cur)
        case .holdStand: stepHold(dt, cur)
        case .ride: stepRide(dt, cur)
        }
        enforceBounds(prev)
        drawOffset = drawOffset * exp(-16 * dt)

        // Body acceleration drives the inertia drag on the head and hands.
        let drawn = pos + drawOffset
        let v = (drawn - lastPos) * (1 / dt)
        if (drawn - lastPos).len < 60 {
            accel = lerp(accel, clampLen((v - lastVel) * (1 / dt), 30000), min(1, 30 * dt))
            lastVel = v
        }
        lastPos = drawn
        let dragWorld = clampLen(accel * -0.00032, 5)
        let drag = P(dragWorld.x * facing, dragWorld.y)
        let (target, boost) = targetPose(cur)
        displayPose = rig.update(target, dt, boost: boost, drag: drag)
    }

    func render(_ origins: [CGPoint], dt: CGFloat) {
        var root = pos + drawOffset
        if mode == .ride && hopT > 0 { root.y += sin(.pi * (1 - hopT / 0.35)) * 10 }
        let (near, far, head) = figurePaths(displayPose, root, facing)
        // Motion trails on fast moves and spins.
        let speed = (root - lastRoot).len / max(dt, 1e-3)
        let spin = abs(angleDiff(lastRot, displayPose.rot)) / max(dt, 1e-3)
        lastRoot = root; lastRot = displayPose.rot
        let want = max(clampf((speed - 330) / 250, 0, 1), clampf((spin - 7) / 6, 0, 1))
        smear = lerp(smear, want, min(1, (want > smear ? 20 : 6) * dt))
        let combined = CGMutablePath()
        combined.addPath(near); combined.addPath(far); combined.addPath(head)
        ghostPaths.insert(combined, at: 0)
        if ghostPaths.count > 7 { ghostPaths.removeLast() }
        for (i, o) in origins.enumerated() where i < layers.count {
            var tr = CGAffineTransform(translationX: -o.x, y: -o.y)
            layers[i].near.path = near.copy(using: &tr)
            layers[i].far.path = far.copy(using: &tr)
            layers[i].head.path = head.copy(using: &tr)
            for (g, gl) in layers[i].ghosts.enumerated() {
                let idx = 2 + g * 2
                if smear > 0.02 && idx < ghostPaths.count {
                    gl.path = ghostPaths[idx].copy(using: &tr)
                    gl.opacity = Float(smear * [0.32, 0.18, 0.09][g])
                } else {
                    gl.path = nil
                }
            }
        }
    }

    /// Moves him without a visible pop: the drawn figure eases from the old spot.
    func setPos(_ p: CGPoint) {
        drawOffset = drawOffset + (pos - p)
        pos = p
    }

    func respawn(_ c: CGPoint) {
        guard !world.screens.isEmpty else { return }
        let s = world.screens[world.screenIndex(at: c) ?? world.nearestScreen(to: c)]
        pos = P(clampf(c.x + rnd(-200...200), s.visible.minX + 30, s.visible.maxX - 30), s.visible.maxY - 40)
        drawOffset = .zero
        launch(vx: 0, vy: 0)
        grabCooldown = 0.4
    }

    /// Keeps him on the screens: side walls, floor and ceiling are solid.
    func enforceBounds(_ prev: CGPoint) {
        guard !world.screens.isEmpty else { return }
        if !pos.x.isFinite || !pos.y.isFinite { respawn(world.screens[0].visible.origin); return }
        if mode == .hang || mode == .ride || mode == .holdStand { return }
        if world.screenIndex(at: pos) == nil {
            let si = world.screenIndex(at: prev) ?? world.nearestScreen(to: pos)
            let v = world.screens[si].visible
            if pos.x < v.minX + 2 || pos.x > v.maxX - 2 {
                pos.x = clampf(pos.x, v.minX + 2, v.maxX - 2)
                if mode == .air { vel.x = -vel.x * 0.25 } else { vel.x = 0 }
            }
            if pos.y < v.minY {
                pos.y = v.minY
                if mode == .air, let fi = world.segs.firstIndex(where: { $0.owner == -(si + 1) }) { land(fi) }
            }
        }
        if mode == .air, let si = world.screenIndex(at: pos) {
            let top = world.screens[si].frame.maxY - 2 - 36
            if pos.y > top { pos.y = top; vel.y = min(vel.y, 0) }
        }
    }

    func launch(vx: CGFloat, vy: CGFloat) {
        mode = .air
        vel = P(vx, vy)
        airT = 0; peakY = pos.y; launchY = pos.y
        ignoreAboveY = nil; wallIntent = nil; jumpTarget = nil
        trick = .none; intent = .none
        skidding = false; antT = 0; pending = nil
        if abs(vx) > 10 { facing = vx > 0 ? 1 : -1 }
    }

    /// Jump from the ground. Standing jumps get a short wind-up crouch first (anticipation);
    /// running jumps go straight away so he never stalls mid-run.
    func takeoff(vx: CGFloat, vy: CGFloat, target: CGPoint?, intent: Intent, styled: Bool) {
        plan = nil
        if mode == .ground && abs(vel.x) < 80 && antT <= 0 {
            antT = intent == .grab ? 0.06 : 0.085
            pending = (vx, vy, target, intent, styled)
            if abs(vx) > 10 { facing = vx > 0 ? 1 : -1 }
            vel.x = 0
            return
        }
        performTakeoff(vx: vx, vy: vy, target: target, intent: intent, styled: styled)
    }

    func performTakeoff(vx: CGFloat, vy: CGFloat, target: CGPoint?, intent: Intent, styled: Bool) {
        launch(vx: vx, vy: vy)
        jumpTarget = target
        self.intent = intent
        let air = 2 * vy / GRAV
        if styled && intent == .none && air > 0.42 && chance(pers.flair * 0.55) {
            let r = rnd(0...1)
            trick = r < 0.55 ? .front : (r < 0.8 ? .twist : (abs(vx) < 140 ? .back : .front))
            trickDur = air * 0.85
        }
    }

    func fall() { launch(vx: vel.x, vy: 0) }

    func faceToward(_ x: CGFloat) { if abs(x - pos.x) > 6 { facing = x > pos.x ? 1 : -1 } }

    // MARK: Ground

    /// Moves toward tx. `carry` keeps full speed through the target (a running take-off).
    @discardableResult
    func runToward(_ tx0: CGFloat, _ seg: Seg, carry: Bool, clampToSeg: Bool = true, maxSpeed: CGFloat? = nil, _ dt: CGFloat) -> Bool {
        let tx = clampToSeg ? clampf(tx0, seg.x0 + 1.5, seg.x1 - 1.5) : tx0
        let d = tx - pos.x
        let dir: CGFloat = d >= 0 ? 1 : -1
        let top = maxSpeed ?? (abs(d) > 220 ? SPRINT_SPEED : RUN_SPEED) * pers.speed
        var desired: CGFloat
        if carry {
            if abs(d) <= abs(vel.x) * dt + 1.5 && vel.x * dir >= 0 { pos.x = tx; return true }
            desired = dir * top
        } else {
            if abs(d) < 2 && abs(vel.x) < 45 { pos.x = tx; vel.x = 0; skidding = false; return true }
            desired = clampf(d * 6, -top, top)
        }
        if vel.x * desired < 0 && abs(vel.x) > 150 {
            skidding = true
            vel.x = approach(vel.x, 0, SKID_DECEL * dt)
        } else {
            skidding = false
            vel.x = approach(vel.x, desired, GROUND_ACCEL * dt)
        }
        let move = vel.x * dt
        pos.x += move
        runPhase += abs(move) / Poses.cycleLength(abs(vel.x))
        if !skidding && abs(vel.x) > 8 { facing = vel.x > 0 ? 1 : -1 }
        return false
    }

    func stepGround(_ dt: CGFloat, _ cur: CursorState) {
        guard let si = world.segIndex(owner: gOwner, x: pos.x, y: pos.y) else { fall(); return }
        let seg = world.segs[si]
        if antT > 0 {
            antT -= dt
            if antT <= 0, let p = pending { performTakeoff(vx: p.vx, vy: p.vy, target: p.target, intent: p.intent, styled: p.styled) }
            return
        }
        if rollT > 0 {
            rollT -= dt
            pos.x += vel.x * dt
            if rollT <= 0 { vel.x *= 0.7 }
            return
        }
        if heroT > 0 { heroT -= dt; return }
        if stumbleT > 0 {
            stumbleT -= dt
            vel.x = approach(vel.x, 0, 700 * dt)
            pos.x = clampf(pos.x + vel.x * dt, seg.x0 + 1, seg.x1 - 1)
            return
        }
        if landT > 0 {
            landT -= dt
            vel.x = approach(vel.x, 0, 900 * dt)
            pos.x = clampf(pos.x + vel.x * dt, seg.x0 + 1, seg.x1 - 1)
            return
        }
        if preDropT > 0 {
            preDropT -= dt
            if preDropT <= 0 {
                let catching = dropCatching
                launch(vx: 0, vy: 60)
                ignoreAboveY = pos.y
                intent = catching ? .grab : .none
            }
            return
        }
        let c = cur.p, dx = c.x - pos.x, h = c.y - pos.y
        if grabCooldown <= 0 && abs(dx) < 10 && h >= 20 && h <= HAND_REACH + 3 {
            mode = .holdStand; plan = nil; emote = .none; faceToward(c.x); return
        }
        let stale: Bool = {
            guard let p = plan else { return true }
            if p.version != world.version && t - p.made > 0.25 { return true }
            if t - p.made > 0.9 { return true }
            return (p.cursor - c).len > 28 && t - p.made > 0.22
        }()
        if stale {
            let wasFrustrated = plan?.action.isFrustrated ?? false
            if let r = world.plan(seg: si, x: pos.x, cursor: c, prevSig: lastSig, climbMult: styleClimb) {
                if r.2 != lastSig { edgeHop = chance(pers.flair * 0.5) }
                plan = Plan(action: r.0, points: r.1, made: t, cursor: c, version: world.version)
                lastSig = r.2
                if !r.0.isFrustrated { emote = .none }
                else if !wasFrustrated { emote = .none; emoteT = rnd(0.2...0.6) }
            } else { plan = nil }
        }
        guard let p = plan else { runToward(pos.x, seg, carry: false, dt); return }
        switch p.action {
        case .walk:
            runToward(c.x, seg, carry: false, dt)
        case .frustrated(let x):
            frustrated(x, seg, cur, dt)
        case .jump(let fx, let to, let q, let ledge):
            if naturalJump(to, q, ledge) { return }
            if runToward(fx, seg, carry: true, dt) { preciseJump(to, q, ledge) }
        case .drop(let fx):
            let nearLeft = fx - seg.x0 < 12, nearRight = seg.x1 - fx < 12
            if nearLeft || nearRight {
                // Run straight off the end; sometimes with a little hop.
                let dir: CGFloat = nearLeft ? -1 : 1
                let edgeX = nearLeft ? seg.x0 : seg.x1
                if edgeHop && abs(edgeX - pos.x) < 14 && vel.x * dir > 100 {
                    performTakeoff(vx: vel.x, vy: 330, target: nil, intent: .none, styled: true); return
                }
                runToward(edgeX + dir * 30, seg, carry: true, clampToSeg: false, dt)
            } else if runToward(fx, seg, carry: false, dt) {
                preDropT = 0.08; dropCatching = false
            }
        case .dropCatch(let fx):
            if runToward(fx, seg, carry: false, dt) { preDropT = 0.06; dropCatching = true }
        case .climb(_, let w, let leap):
            if runToward(w.baseX, seg, carry: true, dt) { startWall(w, leap: leap) }
        case .catchJump:
            if catchLeap(cur) { return }
            let lead = clampf(cur.v.x * 0.25, -120, 120)
            if runToward(c.x + lead, seg, carry: false, dt), abs(dx) < 8 { _ = catchLeap(cur, force: true) }
        }
    }

    /// Take off mid-run if a jump at the current speed already lands on the target.
    func naturalJump(_ to: CGPoint, _ q: Seg, _ ledge: Bool) -> Bool {
        guard abs(vel.x) > 110, vel.x * (to.x - pos.x) > 0 else { return false }
        let dy = q.y - pos.y
        let vx = vel.x
        if ledge {
            let apex = min(JUMP_MAX, max(dy - HAND_REACH + 10, 12))
            let vy = (2 * GRAV * apex).squareRoot()
            let xA = pos.x + vx * vy / GRAV
            guard q.contains(xA, -3) else { return false }
            performTakeoff(vx: vx, vy: vy, target: P(xA, q.y), intent: .none, styled: false)
            return true
        }
        let apex = dy > 0 ? dy + 14 : 12 + min(abs(dy) * 0.05, 14)
        guard apex <= JUMP_MAX else { return false }
        let vy = (2 * GRAV * apex).squareRoot()
        let tt = vy / GRAV + (2 * (apex - dy) / GRAV).squareRoot()
        let xl = pos.x + vx * tt
        guard xl > q.x0 + 6 && xl < q.x1 - 6 else { return false }
        guard !world.trajectoryBlocked(x0: pos.x, y0: pos.y, vx: vx, vy: vy, tLand: tt, target: q) else { return false }
        performTakeoff(vx: vx, vy: vy, target: P(xl, q.y), intent: .none, styled: true)
        return true
    }

    func preciseJump(_ to: CGPoint, _ q: Seg, _ ledge: Bool) {
        let dx = to.x - pos.x, dy = to.y - pos.y
        if ledge {
            let apex = min(JUMP_MAX, max(dy - HAND_REACH + 10, 12))
            let vy = (2 * GRAV * apex).squareRoot()
            takeoff(vx: clampf(dx / (vy / GRAV), -AIR_VX_MAX, AIR_VX_MAX), vy: vy, target: to, intent: .none, styled: false)
            return
        }
        guard let j = solveJump(dx: dx, dy: dy) else { plan = nil; return }
        takeoff(vx: j.vx, vy: j.vy, target: to, intent: .none, styled: true)
    }

    /// Leap at the cursor when the timing works from the current run.
    func catchLeap(_ cur: CursorState, force: Bool = false) -> Bool {
        guard grabCooldown <= 0 else { return false }
        let c = cur.p, dx = c.x - pos.x, h = c.y - pos.y
        if h < 22 {
            guard let j = solveJump(dx: dx, dy: max(h, 0)) else { return false }
            guard force || abs(dx) < 30 || abs(j.vx - vel.x) < 80 else { return false }
            takeoff(vx: j.vx, vy: j.vy, target: nil, intent: .hop, styled: false)
            return true
        }
        let need = h - HAND_REACH + 4
        guard need <= JUMP_MAX + 2 else { return false }
        let apex = clampf(need + 4, 10, JUMP_MAX)
        let vy = (2 * GRAV * apex).squareRoot()
        let tA = vy / GRAV
        let vxNeeded = dx / tA
        guard abs(vxNeeded) <= AIR_VX_MAX else { return false }
        guard force || abs(dx) < 14 || abs(vxNeeded - vel.x) < 70 + AIR_ACCEL * tA * 0.5 else { return false }
        takeoff(vx: vxNeeded, vy: vy, target: nil, intent: .grab, styled: false)
        return true
    }

    // MARK: Idle moods

    func frustrated(_ x: CGFloat, _ seg: Seg, _ cur: CursorState, _ dt: CGFloat) {
        let c = cur.p
        if emote == .pace {
            if runToward(x + paceDir * 22, seg, carry: false, maxSpeed: 45, dt) { paceDir = -paceDir }
        } else {
            if !runToward(x, seg, carry: false, dt) { return }
            if emote != .sit && emote != .pushup && emote != .dance { faceToward(c.x) }
        }
        emoteT -= dt
        guard emoteT <= 0 else { return }
        let above = c.y - pos.y > 30, near = abs(c.x - pos.x) < 160
        let options: [(Emote?, CGFloat)] = above && near
            ? [(.fist, 3), (nil, 3), (.lookUp, 2), (.cross, 1.5), (.pace, 1), (.wave, 1)]
            : [(.cross, 2), (.sit, 2), (.pace, 2), (.wave, 1), (.pushup, 0.8), (.stretch, 1), (.dance, 0.8), (.lookUp, 1)]
        var r = rnd(0...options.reduce(0) { $0 + $1.1 })
        var pick: Emote? = .cross
        for o in options { r -= o.1; if r <= 0 { pick = o.0; break } }
        guard let e = pick else {
            // An optimistic jump at it anyway.
            emote = .none; emoteT = rnd(0.6...1.2)
            let vy = (2 * GRAV * JUMP_MAX).squareRoot()
            takeoff(vx: clampf((c.x - pos.x) / (vy / GRAV), -120, 120), vy: vy, target: nil, intent: .grab, styled: false)
            return
        }
        emote = e
        let long = e == .sit || e == .pushup || e == .dance
        emoteT = (long ? rnd(2.5...4.5) : rnd(1.2...2.6)) * pers.patience
    }

    // MARK: Air

    func stepAir(_ dt: CGFloat, _ cur: CursorState) {
        airT += dt
        let prev = pos
        let c = cur.p
        if intent != .none {
            let tLeft = max(vel.y / GRAV, 0.12)
            let want = clampf((c.x - pos.x) / tLeft, -AIR_VX_MAX, AIR_VX_MAX)
            vel.x = approach(vel.x, want, AIR_ACCEL * dt)
        } else if let tg = jumpTarget {
            let disc = vel.y * vel.y + 2 * GRAV * (pos.y - tg.y)
            if disc > 0 {
                let tl = max((vel.y + disc.squareRoot()) / GRAV, 0.08)
                vel.x = approach(vel.x, clampf((tg.x - pos.x) / tl, -AIR_VX_MAX, AIR_VX_MAX), AIR_ACCEL * 0.6 * dt)
            }
        }
        vel.y -= GRAV * dt
        pos = pos + vel * dt
        peakY = max(peakY, pos.y)

        if grabCooldown <= 0 {
            let hp = P(facing * 1.5, HAND_REACH - 4)
            if segPointDist(prev + hp, pos + hp, c) < GRAB_RADIUS + (intent == .grab ? 6 : 0) { startHang(cur); return }
        }
        if let w = wallIntent {
            if pos.y + HAND_REACH >= w.y0 + 6 { pos.x = w.baseX; enterClimb(w); return }
            if vel.y < -120 { wallIntent = nil }
        }
        // Catch an edge he's come up short of and pull up onto it.
        if ignoreAboveY == nil && vel.y < 140 && airT > 0.08 {
            let hx = pos.x + facing * 4, handY = pos.y + HAND_REACH - 2
            for i in world.segsInY(max(launchY + 8, pos.y + 12), handY) where world.segs[i].contains(hx, 3) {
                let s = world.segs[i]
                startLedge(s, x: clampf(hx, s.x0 + 2, s.x1 - 2)); return
            }
        }
        if vel.y <= 0 {
            if grabCooldown <= 0 && abs(pos.x - c.x) < 7 && prev.y >= c.y - 3 && pos.y <= c.y + 3 {
                setPos(c); startRide(); return
            }
            var best: Int? = nil
            for i in world.segsInY(pos.y, prev.y + 0.5) where world.segs[i].contains(pos.x, 2) {
                let s = world.segs[i]
                if let ig = ignoreAboveY, s.y >= ig - 1 { continue }
                if best == nil || s.y > world.segs[best!].y { best = i }
            }
            if let i = best { land(i); return }
        }
        if let ig = ignoreAboveY, pos.y < ig - 6 { ignoreAboveY = nil }
    }

    func land(_ i: Int) {
        let s = world.segs[i]
        let fallH = peakY - s.y
        let vx = vel.x
        pos.y = s.y
        mode = .ground
        gOwner = s.owner; gRect = world.rects[s.owner]
        trick = .none; intent = .none; wallIntent = nil; ignoreAboveY = nil; jumpTarget = nil
        plan = nil
        vel = P(vx * 0.85, 0)
        if abs(vx) > 20 { facing = vx > 0 ? 1 : -1 }
        if dizzy {
            dizzy = false
            stumbleT = 1.3; vel.x *= 0.5          // spun too much: staggers about
        } else if fallH > 150 && abs(vx) > 80 && chance(0.35 + pers.flair * 0.5) {
            rollT = 0.36
            vel.x = facing * clampf(abs(vx), 110, 220)
        } else if fallH > 150 && chance(pers.flair * 0.3) {
            heroT = 0.5; vel.x = 0
        } else if abs(vx) > 180 && chance(0.04) {
            stumbleT = 0.35
        } else if abs(vx) > 110 && fallH < 90 {
            landT = 0.03; landDepth = 0.35          // absorb and keep running
        } else {
            landDepth = clampf(fallH / 100, 0.25, 1)
            landT = clampf(fallH / 800, 0.05, 0.18)
        }
    }

    // MARK: Walls

    func startWall(_ w: Wall, leap: Bool) {
        plan = nil
        pos.x = w.baseX
        facing = w.side
        leapIntent = leap
        wall = w; wallRect = world.rects[w.owner]
        if abs(vel.x) > 130 && vel.x * w.side > 0 && w.y0 <= pos.y + 25 {
            mode = .wallRun
            wallVy = min(WALLRUN_VY, 170 + abs(vel.x) * 0.9)
            vel = .zero
            climbPhase = 0
        } else if pos.y + HAND_REACH < w.y0 + 4 {
            let need = min(w.y0 + 8 - (pos.y + HAND_REACH), JUMP_MAX + 10)
            launch(vx: 0, vy: (2 * GRAV * need).squareRoot())
            facing = w.side
            wallIntent = w
        } else {
            enterClimb(w)
        }
    }

    func enterClimb(_ w: Wall) {
        mode = .climb
        wall = w; wallRect = world.rects[w.owner]
        vel = .zero; climbStall = 0
        facing = w.side; pos.x = w.baseX
        wallIntent = nil
        dyno = w.hasTop && chance(pers.flair * 0.7)
    }

    /// Hands reached the top of the wall: pull up, or push off if there's nothing to stand on.
    func topOut(_ w: Wall) {
        if let ti = world.topSeg(of: w) {
            let s = world.segs[ti]
            edge = P(w.x, s.y); edgeOwner = s.owner; edgeRect = world.rects[s.owner]
            beginMantle()
        } else {
            launch(vx: -w.side * 200, vy: 400)
            trick = .back; trickDur = 0.42
        }
    }

    func wallLeapCheck(_ w: Wall, _ c: CGPoint) -> Bool {
        guard leapIntent else { return false }
        let ddx = (c.x - w.x) * -w.side
        let handY = pos.y + HAND_REACH
        let atTop = handY >= w.y1 - 1
        guard ddx > 12 && ddx < LEAP_RANGE + 20 && (handY >= c.y - 4 || atTop) && abs(c.y - handY) < 110 else { return false }
        let dx = c.x - pos.x
        let tt = clampf(abs(dx) / 380, 0.2, 0.75)
        let vy = clampf((c.y - handY + 0.5 * GRAV * tt * tt) / tt, -250, 750)
        leapIntent = false
        launch(vx: clampf(dx / tt, -460, 460), vy: vy)
        intent = .grab
        return true
    }

    func stepWallRun(_ dt: CGFloat, _ cur: CursorState) {
        guard let w = wall else { fall(); return }
        pos.x = w.baseX
        facing = w.side
        if grabCooldown <= 0 && (P(w.x, pos.y + HAND_REACH - 4) - cur.p).len < GRAB_RADIUS + 5 { startHang(cur); return }
        if wallLeapCheck(w, cur.p) { return }
        wallVy -= WALLRUN_GRAV * dt
        let dy = wallVy * dt
        pos.y += dy
        climbPhase += abs(dy) / (24 * S)
        if pos.y + HAND_REACH >= w.y1 - 1 { pos.y = w.y1 - HAND_REACH + 1; topOut(w); return }
        if wallVy <= 30 { enterClimb(w) }
    }

    func stepClimb(_ dt: CGFloat, _ cur: CursorState) {
        guard let w = wall else { fall(); return }
        pos.x = w.baseX
        facing = w.side
        let c = cur.p
        if grabCooldown <= 0 && (P(w.x, pos.y + HAND_REACH - 4) - c).len < GRAB_RADIUS + 6 { startHang(cur); return }
        if wallLeapCheck(w, c) { return }
        let remaining = w.y1 - (pos.y + HAND_REACH - 1)
        let stuck = !w.hasTop && climbStall > 0
        if dyno && remaining < 38 && remaining > 14 {
            // Spring for the top instead of climbing the last bit.
            launch(vx: 0, vy: (2 * GRAV * (remaining + 12)).squareRoot())
            facing = w.side
            return
        }
        if remaining > 1 && !stuck {
            // Pull hard mid-reach, slow as the next hand goes up: speed follows the hand cycle.
            let surge = 0.35 + 1.3 * pow(sin(.pi * frac(climbPhase * 2)), 2)
            let dy = min(CLIMB_SPEED * pers.speed * surge * dt, remaining)
            pos.y += dy
            climbPhase += dy / (28 * S)
        } else {
            climbStall += dt
            if climbStall > 0.05 { topOut(w) }
        }
    }

    func startLedge(_ s: Seg, x: CGFloat) {
        mode = .ledge
        edge = P(x, s.y); edgeOwner = s.owner; edgeRect = world.rects[s.owner]
        ledgeT = rnd(0.04...0.28) * pers.patience
        setPos(edge - P(facing * 6.5 * S, 60 * S))
        vel = .zero
        trick = .none; intent = .none
    }

    func stepLedge(_ dt: CGFloat) {
        ledgeT -= dt
        if ledgeT <= 0 { beginMantle() }
    }

    func beginMantle() {
        mode = .mantle
        mantleT = 0
        mantleDur = 0.32 / pers.speed
        mantleFrom = pos
        let tx = edge.x + facing * MANTLE_IN
        let s = world.segNear(x: edge.x, y: edge.y, tol: 2).map { world.segs[$0] }
        mantleTo = P(s.map { clampf(tx, $0.x0 + 2, $0.x1 - 2) } ?? tx, edge.y)
    }

    func stepMantle(_ dt: CGFloat) {
        mantleT += dt / mantleDur
        if mantleT >= 1 {
            pos = mantleTo
            if let si = world.segIndex(owner: edgeOwner, x: pos.x, y: pos.y) ?? world.segNear(x: pos.x, y: pos.y, tol: 3) {
                mode = .ground
                pos.y = world.segs[si].y
                gOwner = world.segs[si].owner; gRect = world.rects[gOwner]
                landT = 0.02; landDepth = 0.3; plan = nil; vel = P(facing * 40, 0)
            } else {
                fall()
            }
            return
        }
        let midY = edge.y - 14
        if mantleT < 0.5 {
            pos = P(mantleFrom.x, lerp(mantleFrom.y, midY, ease(mantleT / 0.5)))
        } else {
            let e = ease((mantleT - 0.5) / 0.5)
            pos = P(lerp(mantleFrom.x, mantleTo.x, e), lerp(midY, mantleTo.y, e))
        }
    }

    // MARK: Caught the cursor

    func startHang(_ cur: CursorState) {
        let c = cur.p
        omega = clampf((vel.x - cur.v.x) / HANG_LEN, -7, 7)
        theta = 0
        mode = .hang
        setPos(P(c.x, c.y - 62 * S))
        trick = .none; intent = .none; wallIntent = nil; leapIntent = false; plan = nil
        stillT = 0; trickT = rnd(1.2...2.5); spins = 0; grip = GRAV
    }

    func stepHang(_ dt: CGFloat, _ cur: CursorState) {
        let c = cur.p
        stillT = cur.speed < 60 ? stillT + dt : 0
        pullT = max(0, pullT - dt); kickT = max(0, kickT - dt)
        if stillT > 1.0 && abs(omega) < 2 {
            trickT -= dt
            if trickT <= 0 {
                trickT = rnd(1.6...3.2) * pers.patience
                let r = rnd(0...1)
                if r < 0.35 { pullT = 1.8 }
                else if r < 0.6 { kickT = 1.0 }
                else if r < 0.85 { omega += (omega >= 0 ? 1 : -1) * rnd(2.5...3.5) }
                else if chance(pers.flair) { setPos(c); startRide(); return }
            }
        }
        // A real pendulum hanging from the moving cursor: move the mouse in circles near his
        // natural swing rate (about once a second) and he goes right over the top.
        let a = clampLen(cur.a, 30000)
        let alpha = (-a.x * cos(theta) - (GRAV + a.y) * sin(theta)) / HANG_LEN - 0.8 * omega
        omega = clampf(omega + alpha * dt, -18, 18)
        theta += omega * dt
        if theta > .pi { theta -= 2 * .pi; spins += 1 } else if theta < -.pi { theta += 2 * .pi; spins += 1 }
        spins = max(0, spins - dt * 0.15)
        // He lets go when the pull on his arms beats his grip (a sharp flick), not just because
        // the cursor is moving fast.
        let tension = HANG_LEN * omega * omega + (GRAV + a.y) * cos(theta) - a.x * sin(theta)
        grip = lerp(grip, tension, min(1, 25 * dt))
        if grip > 22000 || cur.speed > 4500 { release(cur); return }
        pos = P(c.x, c.y - 62 * S)
        // Feet can find a ledge only when he's hanging calmly, not mid-swing.
        guard abs(omega) < 3 && abs(theta) < 0.6 else { return }
        let feet = P(c.x + HANG_LEN * sin(theta), c.y - HANG_LEN * cos(theta))
        var best: Int? = nil
        for i in world.segsInY(feet.y, c.y - 14) where world.segs[i].contains(feet.x, 1) {
            if best == nil || world.segs[i].y > world.segs[best!].y { best = i }
        }
        if let i = best {
            let s = world.segs[i]
            mode = .holdStand
            gOwner = s.owner; gRect = world.rects[s.owner]
            setPos(P(feet.x, s.y))
            vel = .zero
        }
    }

    func release(_ cur: CursorState) {
        let c = cur.p
        let feet = P(c.x + HANG_LEN * sin(theta), c.y - HANG_LEN * cos(theta))
        var v = cur.v * 0.85 + P(cos(theta), sin(theta)) * (omega * HANG_LEN)
        if v.len > 1900 { v = v * (1900 / v.len) }
        setPos(feet)
        launch(vx: v.x, vy: v.y)
        // Keep tumbling the way he was spinning. World spin = facing * local rotation.
        if abs(omega) > 5 {
            trick = omega * facing > 0 ? .back : .front
            trickDur = clampf(2 * .pi / abs(omega), 0.3, 0.7)
        } else if v.len > 800 {
            trick = chance(0.5) ? .front : .back; trickDur = 0.55
        }
        dizzy = spins >= 2
        spins = 0; grip = 0
        grabCooldown = 1.0
    }

    func stepHold(_ dt: CGFloat, _ cur: CursorState) {
        let c = cur.p
        guard world.segIndex(owner: gOwner, x: pos.x, y: pos.y) != nil else { startHang(cur); return }
        if cur.speed > 3000 { mode = .ground; grabCooldown = 1.0; return }
        let h = c.y - pos.y
        if h > HANG_LEN + 2 { startHang(cur); return }
        if h < 17 { mode = .ground; grabCooldown = 0.3; return }
        if (c.x - pos.x) * facing < -6 { facing = -facing }
        vel.x = clampf((c.x - facing * 5 - pos.x) * 14, -RUN_SPEED * 1.3, RUN_SPEED * 1.3)
        pos.x += vel.x * dt
        runPhase += abs(vel.x * dt) / Poses.cycleLength(abs(vel.x))
    }

    func startRide() {
        mode = .ride
        vel = .zero
        trick = .none; intent = .none; plan = nil
        rideT = 0; stillT = 0; trickT = rnd(1.2...2.5)
    }

    func stepRide(_ dt: CGFloat, _ cur: CursorState) {
        if cur.speed > RIDE_FLING {
            pos = cur.p
            launch(vx: cur.v.x * 0.7, vy: max(cur.v.y * 0.7, 120))
            if cur.speed > 2400 { trick = .front; trickDur = 0.5 }
            grabCooldown = 1.0
            return
        }
        pos = cur.p
        rideT += dt
        hopT = max(0, hopT - dt)
        stillT = cur.speed < 60 ? stillT + dt : 0
        if abs(cur.v.x) > 50 { facing = cur.v.x > 0 ? 1 : -1 }
        if stillT > 0.8 {
            trickT -= dt
            if trickT <= 0 {
                trickT = rnd(1.5...3) * pers.patience
                if chance(0.55) { hopT = 0.35 }
                else if rideT > 4 && chance(0.45) { startHang(cur); omega = (chance(0.5) ? 1 : -1) * 3; return }
            }
        }
    }

    // MARK: Surfaces changed

    func worldChanged() {
        switch mode {
        case .ground, .holdStand:
            if gOwner >= 0, let r = world.rects[gOwner], let old = gRect {
                pos.x += r.minX - old.minX
                pos.y = r.maxY
                gRect = r
            }
            if world.segIndex(owner: gOwner, x: pos.x, y: pos.y) != nil { return }
            // Detected edges are re-found by position each frame; windows keep their id.
            if gOwner < 0, let i = world.segNear(x: pos.x, y: pos.y, tol: 4) {
                pos.y = world.segs[i].y; gOwner = world.segs[i].owner; gRect = world.rects[gOwner]; return
            }
            dropOff()
        case .climb, .wallRun:
            guard let w = wall else { return }
            if w.owner >= 0 {
                guard let r = world.rects[w.owner], let old = wallRect else { fall(); return }
                pos.x += (w.side > 0 ? r.minX - old.minX : r.maxX - old.maxX)
                pos.y += r.maxY - old.maxY
                wallRect = r
            }
            let x = w.x + (w.owner >= 0 ? pos.x - w.baseX : 0)
            if let nw = world.walls.first(where: { $0.side == w.side && abs($0.x - x) <= 4 && pos.y + HAND_REACH >= $0.y0 - 6 && pos.y + 20 <= $0.y1 &&
                                                   ($0.owner == w.owner || w.owner < 0) }) {
                wall = nw; pos.x = nw.baseX
            } else {
                fall()
            }
        case .ledge, .mantle:
            if edgeOwner >= 0 {
                guard let r = world.rects[edgeOwner], let old = edgeRect else { fall(); return }
                let d = P(r.minX - old.minX, r.maxY - old.maxY)
                edge = edge + d; pos = pos + d; mantleFrom = mantleFrom + d; mantleTo = mantleTo + d
                edgeRect = r
            } else if mode == .ledge && world.segNear(x: edge.x, y: edge.y, tol: 4) == nil {
                fall()
            }
        default: break
        }
    }

    private func dropOff() {
        if mode == .holdStand { mode = .hang; theta = 0; omega = 0 } else { fall() }
    }

    // MARK: Pose selection

    /// The pose he's aiming for this instant, and how snappy the rig should be about reaching it.
    func targetPose(_ cur: CursorState) -> (Pose, CGFloat) {
        let local = P((cur.p.x - pos.x) * facing / S, (cur.p.y - pos.y) / S)
        switch mode {
        case .ground:
            if antT > 0 { return (Poses.windup, 1.6) }
            if rollT > 0 {
                var p = Poses.roll
                p.rot = -2 * .pi * ease(1 - rollT / 0.36)
                return (p, 1.8)
            }
            if heroT > 0 { return (heroT > 0.15 ? Poses.hero : Poses.stand(t), heroT > 0.15 ? 1.6 : 0.8) }
            if stumbleT > 0 {
                if stumbleT > 0.35 {
                    // Dizzy: swaying on the spot, head lolling.
                    var p = Poses.stand(t, look: 0.4 * sin(t * 7))
                    p.hL = p.shoulder + P(-10 + 3 * sin(t * 9), -8); p.hR = p.shoulder + P(10 + 3 * cos(t * 8), -7)
                    p.pivot = P(0, 0); p.rot = 0.28 * sin(t * 5.5)
                    return (p, 0.9)
                }
                return (Poses.stumble(t), 1.2)
            }
            if landT > 0 { return (Poses.crouch(landDepth), 1.7) }
            if preDropT > 0 { return (Poses.crouch(0.6), 1.5) }
            if skidding { return (Poses.skid, 1.2) }
            if abs(vel.x) > 12 {
                return (Poses.locomote(runPhase, abs(vel.x)), 1.4)
            }
            switch emote {
            case .fist: return (Poses.fist(t), 1)
            case .sit: return (Poses.sit(t), 0.6)
            case .cross: return (Poses.cross(t), 0.8)
            case .lookUp: return (Poses.lookUp(t), 0.8)
            case .wave: return (Poses.wave(t), 1)
            case .pushup: return (Poses.pushup(t), 0.9)
            case .stretch: return (Poses.stretch(t), 0.7)
            case .dance: return (Poses.dance(t), 1.1)
            case .none, .pace: break
            }
            let look = clampf(-atan2(local.y - 45, abs(local.x) + 40) * 0.5, -0.5, 0.3)
            return (Poses.stand(t, look: look), 0.8)
        case .air:
            if trick != .none {
                let u = clampf(airT / trickDur, 0, 1)
                switch trick {
                case .front, .back:
                    var p = u < 0.9 ? Poses.tuck : Poses.jump(-0.5, t)
                    p.rot = (trick == .front ? -1 : 1) * 2 * .pi * ease(u)
                    return (p, 2.2)
                case .twist:
                    var p = Poses.jump(clampf(vel.y / 450, -1, 1), t)
                    p.sx = cos(2 * .pi * ease(u))
                    return (p, 1.6)
                case .none: break
                }
            }
            if intent == .grab { return (Poses.reach(local), 1.3) }
            return (Poses.jump(clampf(vel.y / 420, -1, 1), t), 1.1)
        case .wallRun:
            return (Poses.wallRun(climbPhase), 1.8)
        case .climb:
            return (Poses.climb(climbPhase), 1.8)
        case .ledge:
            return (Poses.ledgeHang(sin(t * 7) * 0.5), 1.2)
        case .mantle:
            return (Poses.mantle(mantleT, P((edge.x - pos.x) * facing / S, (edge.y - pos.y) / S)), 1.8)
        case .hang:
            let pull = pullT > 0 ? pow(sin(.pi * frac((1.8 - pullT) / 0.9)), 2) : 0
            let kick = kickT > 0 ? sin(t * 14) : 0.3 * sin(t * 2.6)
            var p = Poses.hang(clampf(-omega * 1.2, -6, 6), pull, kick)
            p.rot = theta * facing
            return (p, 1.2 + min(abs(omega) / 5, 2.5))
        case .holdStand:
            return (Poses.hold(t, local, runPhase, abs(vel.x)), 1.3)
        case .ride:
            return (Poses.ride(t), 1)
        }
    }
}

// MARK: - Overlay

final class Overlay {
    let window: NSWindow
    let view: NSView
    let origin: CGPoint
    let segLayer = CAShapeLayer(), wallLayer = CAShapeLayer(), planLayer = CAShapeLayer()

    init(screen: NSScreen) {
        origin = screen.frame.origin
        window = NSWindow(contentRect: screen.frame, styleMask: .borderless, backing: .buffered, defer: false)
        window.isOpaque = false
        window.backgroundColor = .clear
        window.hasShadow = false
        window.ignoresMouseEvents = true
        window.level = .statusBar
        window.collectionBehavior = [.canJoinAllSpaces, .stationary, .fullScreenAuxiliary, .ignoresCycle]
        window.isReleasedWhenClosed = false
        view = NSView(frame: NSRect(origin: .zero, size: screen.frame.size))
        view.wantsLayer = true
        window.contentView = view
        for (l, c) in [(segLayer, NSColor.systemGreen), (wallLayer, NSColor.systemOrange), (planLayer, NSColor.systemBlue)] {
            l.strokeColor = c.withAlphaComponent(0.85).cgColor
            l.fillColor = nil
            l.lineWidth = 1.5
            l.lineCap = .round
            view.layer?.addSublayer(l)
        }
        window.setFrame(screen.frame, display: false)
        window.orderFrontRegardless()
    }

    func makeFigureLayers() -> FigureLayers {
        func mk(_ alpha: CGFloat, fill: Bool) -> CAShapeLayer {
            let l = CAShapeLayer()
            l.frame = view.bounds
            l.strokeColor = fill ? nil : NSColor(white: 1, alpha: alpha).cgColor
            l.fillColor = fill ? NSColor.white.cgColor : nil
            l.lineWidth = LINE_W
            l.lineCap = .round
            l.lineJoin = .round
            l.shadowColor = NSColor.black.cgColor
            l.shadowOffset = CGSize(width: 0, height: -0.5)
            l.shadowRadius = 1.2
            l.shadowOpacity = 0
            view.layer?.addSublayer(l)
            return l
        }
        let ghosts = (0..<3).map { _ -> CAShapeLayer in let g = mk(1, fill: false); g.lineWidth = LINE_W * 0.9; return g }
        let far = mk(0.7, fill: false)
        return FigureLayers(near: mk(1, fill: false), far: far, head: mk(1, fill: true), ghosts: ghosts)
    }
}

// MARK: - App

final class AppDelegate: NSObject, NSApplicationDelegate {
    let world = World()
    var runner: Runner!
    var overlays: [Overlay] = []
    var statusItem: NSStatusItem!
    var visionItem: NSMenuItem!
    var timer: Timer?
    var displayLink: AnyObject?
    var paused = false
    var showSurfaces = false
    var shadow = false
    let vision = Vision()
    let cursor = CursorTracker()
    var lastTick = CACurrentMediaTime()
    var lastWorldRefresh: CFTimeInterval = 0
    var frame = 0
    let logState = ProcessInfo.processInfo.environment["STICKCHASE_LOG"] != nil

    func applicationDidFinishLaunching(_ notification: Notification) {
        // Only one stick figure: a new launch replaces any copy that's already running.
        let me = NSRunningApplication.current
        for other in NSRunningApplication.runningApplications(withBundleIdentifier: BUNDLE_ID) where other != me {
            other.terminate()
        }
        NSApp.setActivationPolicy(.accessory)
        world.refresh(force: true)
        buildOverlays()
        runner = Runner(world: world, cursor: NSEvent.mouseLocation)
        runner.layers = overlays.map { $0.makeFigureLayers() }
        setupStatusItem()
        startClock()
        vision.onUpdate = { [weak self] segs, walls in
            guard let self else { return }
            self.world.setVision(segs: segs, walls: walls)
            self.runner.worldChanged()
        }
        startVision(prompt: true)
        NotificationCenter.default.addObserver(forName: NSApplication.didChangeScreenParametersNotification, object: nil, queue: .main) { [weak self] _ in
            guard let self else { return }
            self.world.refresh(force: true)
            self.buildOverlays()
            self.runner.layers = self.overlays.map { $0.makeFigureLayers() }
            self.applyShadow()
            self.runner.worldChanged()
            self.startClock()
            if self.vision.running { self.vision.stop(); self.startVision(prompt: false) }
        }
    }

    func startVision(prompt: Bool) {
        if !CGPreflightScreenCaptureAccess() {
            if prompt { CGRequestScreenCaptureAccess() }
            updateVisionItem()
            return
        }
        vision.start { [weak self] _ in self?.updateVisionItem() }
    }

    func updateVisionItem() {
        guard visionItem != nil else { return }
        if vision.running {
            visionItem.title = "Screen Vision: On"
            visionItem.isEnabled = false
        } else {
            visionItem.title = "Enable Screen Vision…"
            visionItem.isEnabled = true
        }
    }

    @objc func enableVision(_ sender: Any?) {
        if CGPreflightScreenCaptureAccess() { startVision(prompt: false); return }
        CGRequestScreenCaptureAccess()
        if let url = URL(string: "x-apple.systempreferences:com.apple.preference.security?Privacy_ScreenCapture") {
            NSWorkspace.shared.open(url)
        }
        let alert = NSAlert()
        alert.messageText = "Let Stick Chase see your screen"
        alert.informativeText = "Turn on Stick Chase under Screen & System Audio Recording, then choose Restart Stick Chase from its menu. He'll use buttons, icons, images and panels as ledges and walls. Nothing is saved or sent anywhere."
        alert.runModal()
    }

    @objc func restart(_ sender: Any?) {
        let path = Bundle.main.bundlePath
        let task = Process()
        task.executableURL = URL(fileURLWithPath: "/bin/sh")
        task.arguments = ["-c", "sleep 0.5; open \"\(path)\""]
        try? task.run()
        NSApp.terminate(nil)
    }

    func startClock() {
        timer?.invalidate(); timer = nil
        if #available(macOS 14.0, *), let v = overlays.first?.view {
            (displayLink as? CADisplayLink)?.invalidate()
            let dl = v.displayLink(target: self, selector: #selector(onFrame(_:)))
            dl.add(to: .main, forMode: .common)
            displayLink = dl
        } else {
            let tm = Timer(timeInterval: 1.0 / 60.0, repeats: true) { [weak self] _ in self?.tick() }
            RunLoop.main.add(tm, forMode: .common)
            timer = tm
        }
    }

    @objc func onFrame(_ sender: AnyObject) { tick() }

    func buildOverlays() {
        overlays.forEach { $0.window.orderOut(nil) }
        overlays = NSScreen.screens.map { Overlay(screen: $0) }
    }

    func setupStatusItem() {
        statusItem = NSStatusBar.system.statusItem(withLength: NSStatusItem.variableLength)
        if let img = NSImage(systemSymbolName: "figure.run", accessibilityDescription: "Stick Chase") {
            img.isTemplate = true
            statusItem.button?.image = img
        } else {
            statusItem.button?.title = "🏃"
        }
        let m = NSMenu()
        m.autoenablesItems = false
        m.addItem(withTitle: "Pause", action: #selector(togglePause(_:)), keyEquivalent: "p")
        m.addItem(.separator())
        visionItem = m.addItem(withTitle: "Enable Screen Vision…", action: #selector(enableVision(_:)), keyEquivalent: "")
        m.addItem(withTitle: "Show Surfaces", action: #selector(toggleSurfaces(_:)), keyEquivalent: "d")
        m.addItem(withTitle: "Soft Shadow", action: #selector(toggleShadow(_:)), keyEquivalent: "s")
        m.addItem(.separator())
        m.addItem(withTitle: "Restart Stick Chase", action: #selector(restart(_:)), keyEquivalent: "")
        m.addItem(withTitle: "Quit Stick Chase", action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
        for item in m.items where item.action != #selector(NSApplication.terminate(_:)) { item.target = self }
        statusItem.menu = m
        updateVisionItem()
    }

    @objc func togglePause(_ sender: NSMenuItem) {
        paused.toggle()
        sender.title = paused ? "Resume" : "Pause"
    }

    @objc func toggleShadow(_ sender: NSMenuItem) {
        shadow.toggle()
        sender.state = shadow ? .on : .off
        applyShadow()
    }

    func applyShadow() {
        for l in runner.layers { for s in [l.near, l.far, l.head] { s.shadowOpacity = shadow ? 0.55 : 0 } }
    }

    @objc func toggleSurfaces(_ sender: NSMenuItem) {
        showSurfaces.toggle()
        sender.state = showSurfaces ? .on : .off
        if !showSurfaces { for o in overlays { o.segLayer.path = nil; o.wallLayer.path = nil; o.planLayer.path = nil } }
    }

    func drawSurfaces() {
        let segs = CGMutablePath(), walls = CGMutablePath(), plans = CGMutablePath()
        for s in world.segs { segs.move(to: P(s.x0, s.y)); segs.addLine(to: P(s.x1, s.y)) }
        for w in world.walls { walls.move(to: P(w.x, w.y0)); walls.addLine(to: P(w.x, w.y1)) }
        if let pts = runner.plan?.points, pts.count > 1 { plans.addLines(between: pts) }
        for o in overlays {
            var tr = CGAffineTransform(translationX: -o.origin.x, y: -o.origin.y)
            o.segLayer.path = segs.copy(using: &tr)
            o.wallLayer.path = walls.copy(using: &tr)
            o.planLayer.path = plans.copy(using: &tr)
        }
    }

    func tick() {
        let now = CACurrentMediaTime()
        let dt = CGFloat(min(now - lastTick, 1.0 / 30.0))
        lastTick = now
        cursor.update(NSEvent.mouseLocation, dt)
        if paused { return }
        if now - lastWorldRefresh > 1.0 / 15.0 {
            lastWorldRefresh = now
            let v = world.version
            world.refresh()
            if world.version != v { runner.worldChanged() }
        }
        // Two physics substeps per frame keeps fast moves from tunnelling through ledges.
        runner.step(dt / 2, cursor.s)
        runner.step(dt / 2, cursor.s)
        CATransaction.begin()
        CATransaction.setDisableActions(true)
        runner.render(overlays.map { $0.origin }, dt: dt)
        frame += 1
        if showSurfaces && frame % 8 == 0 { drawSurfaces() }
        CATransaction.commit()
        if logState && frame % 30 == 0 {
            let c = cursor.s.p
            print(String(format: "mode=%@ pos=(%.0f,%.0f) cursor=(%.0f,%.0f) segs=%d walls=%d vision=%d build=%.1fms plan=%@",
                         "\(runner.mode)", runner.pos.x, runner.pos.y, c.x, c.y, world.segs.count, world.walls.count,
                         world.visionSegs.count, world.buildMs,
                         runner.plan.map { "\($0.action)".components(separatedBy: "(").first ?? "" } ?? "-"))
            fflush(stdout)
        }
    }
}

// MARK: - Dev tools

func drawFigure(_ ctx: CGContext, _ p: Pose, _ root: CGPoint, _ facing: CGFloat) {
    let (near, far, head) = figurePaths(p, root, facing)
    ctx.setLineCap(.round); ctx.setLineJoin(.round); ctx.setLineWidth(LINE_W)
    ctx.setStrokeColor(NSColor(white: 1, alpha: 0.7).cgColor); ctx.addPath(far); ctx.strokePath()
    ctx.setStrokeColor(NSColor.white.cgColor); ctx.addPath(near); ctx.strokePath()
    ctx.setFillColor(NSColor.white.cgColor); ctx.addPath(head); ctx.fillPath()
}

func makeCanvas(_ size: NSSize, scale: CGFloat) -> (NSBitmapImageRep, CGContext) {
    let rep = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: Int(size.width * scale), pixelsHigh: Int(size.height * scale),
                               bitsPerSample: 8, samplesPerPixel: 4, hasAlpha: true, isPlanar: false,
                               colorSpaceName: .deviceRGB, bytesPerRow: 0, bitsPerPixel: 0)!
    rep.size = size
    NSGraphicsContext.saveGraphicsState()
    NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: rep)
    return (rep, NSGraphicsContext.current!.cgContext)
}

func saveCanvas(_ rep: NSBitmapImageRep, _ path: String) {
    NSGraphicsContext.restoreGraphicsState()
    try? rep.representation(using: .png, properties: [:])?.write(to: URL(fileURLWithPath: path))
    print("wrote \(path)")
}

/// Pose sheet: key poses, plus the run and walk cycles sampled through the rig.
func renderSnapshot(to path: String) {
    let t: CGFloat = 0.3
    var items: [(String, Pose)] = []
    for i in 0..<8 { items.append(("run \(i)/8", Poses.locomote(CGFloat(i) / 8, RUN_SPEED))) }
    for i in 0..<8 { items.append(("walk \(i)/8", Poses.locomote(CGFloat(i) / 8, 40))) }
    items += [("stand", Poses.stand(t)), ("windup", Poses.windup), ("stretch", Poses.jump(1, t)), ("rise", Poses.jump(0.4, t)),
              ("apex", Poses.jump(0, t)), ("fall", Poses.jump(-0.5, t)), ("fall fast", Poses.jump(-1, t)), ("land", Poses.crouch(1)),
              ("climb 0", Poses.climb(0)), ("climb .25", Poses.climb(0.25)), ("climb .5", Poses.climb(0.5)), ("climb .75", Poses.climb(0.75)),
              ("wallrun", Poses.wallRun(0.2)), ("ledge", Poses.ledgeHang(0.3)), ("skid", Poses.skid), ("reach", Poses.reach(P(14, 62)))]
    let cols = 8, cellW: CGFloat = 80, cellH: CGFloat = 80
    let rows = (items.count + cols - 1) / cols
    let size = NSSize(width: CGFloat(cols) * cellW, height: CGFloat(rows) * cellH)
    let (rep, ctx) = makeCanvas(size, scale: 3)
    ctx.setFillColor(NSColor(white: 0.16, alpha: 1).cgColor)
    ctx.fill(CGRect(origin: .zero, size: size))
    for (i, it) in items.enumerated() {
        let col = i % cols, row = rows - 1 - i / cols
        let root = P(CGFloat(col) * cellW + cellW / 2, CGFloat(row) * cellH + 22)
        ctx.setStrokeColor(NSColor(white: 0.35, alpha: 1).cgColor); ctx.setLineWidth(0.5)
        ctx.move(to: P(root.x - 30, root.y)); ctx.addLine(to: P(root.x + 30, root.y)); ctx.strokePath()
        drawFigure(ctx, it.1, root, 1)
        (it.0 as NSString).draw(at: NSPoint(x: root.x - 30, y: root.y - 16),
                                withAttributes: [.font: NSFont.systemFont(ofSize: 8), .foregroundColor: NSColor(white: 0.7, alpha: 1)])
    }
    saveCanvas(rep, path)
}

/// Runs edge detection on a screenshot and draws what he'd use as ledges (green) and walls (orange).
func visionTest(_ input: String, _ output: String) {
    guard let img = NSImage(contentsOfFile: input), let cg = img.cgImage(forProposedRect: nil, context: nil, hints: nil) else {
        print("can't read \(input)"); return
    }
    // Work at point resolution, like the live capture.
    let w = cg.width / 2, h = cg.height / 2
    var lum = [UInt8](repeating: 0, count: w * h)
    let cs = CGColorSpaceCreateDeviceGray()
    lum.withUnsafeMutableBytes { buf in
        let c = CGContext(data: buf.baseAddress, width: w, height: h, bitsPerComponent: 8, bytesPerRow: w, space: cs, bitmapInfo: 0)!
        c.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
    }
    let t0 = CACurrentMediaTime()
    let e = detectEdges(lum, w, h)
    let ms = (CACurrentMediaTime() - t0) * 1000
    let frame = CGRect(x: 0, y: 0, width: w, height: h)
    let (segs, walls) = surfaces(from: e, imgW: w, imgH: h, frame: frame, lum: lum)
    let (rep, ctx) = makeCanvas(NSSize(width: w, height: h), scale: 1)
    ctx.draw(cg, in: CGRect(x: 0, y: 0, width: w, height: h))
    ctx.setFillColor(NSColor(white: 0, alpha: 0.35).cgColor)
    ctx.fill(CGRect(x: 0, y: 0, width: w, height: h))
    ctx.setLineWidth(2)
    ctx.setStrokeColor(NSColor.systemGreen.cgColor)
    for s in segs { ctx.move(to: P(s.x0, s.y)); ctx.addLine(to: P(s.x1, s.y)) }
    ctx.strokePath()
    ctx.setStrokeColor(NSColor.systemOrange.cgColor)
    for wl in walls { ctx.move(to: P(wl.x, wl.y0)); ctx.addLine(to: P(wl.x, wl.y1)) }
    ctx.strokePath()
    saveCanvas(rep, output)
    print(String(format: "ledges=%d walls=%d detect=%.1fms", segs.count, walls.count, ms))
}

/// Simulates a chase over made-up windows and writes a contact sheet of frames around the runner.
func renderFilm(to path: String, seed: Int) {
    _ = NSApplication.shared
    let w = World()
    let frame = CGRect(x: 0, y: 0, width: 1470, height: 956)
    w.screens = [(frame: frame, visible: CGRect(x: 0, y: 60, width: 1470, height: 863))]
    let wins: [(Int, CGRect)] = [
        (1, CGRect(x: 520, y: 60, width: 420, height: 300)),
        (2, CGRect(x: 180, y: 200, width: 360, height: 260)),
        (3, CGRect(x: 900, y: 150, width: 380, height: 420)),
        (4, CGRect(x: 380, y: 420, width: 300, height: 240)),
    ]
    for (id, r) in wins { w.rects[id] = r }
    w.rebuild(wins)
    let r = Runner(world: w, cursor: P(300, 100))
    r.pos = P(120, 60); r.mode = .ground; r.gOwner = -1
    let cur = CursorTracker()
    let script: [(CGFloat, CGFloat, CGFloat)] = seed == 0
        ? [(700, 120, 2.5), (620, 470, 4), (1080, 640, 5), (300, 560, 4)]
        : [(1000, 300, 3), (450, 700, 5), (200, 90, 3), (1200, 800, 4)]
    var frames: [(Pose, CGPoint, CGFloat, CGPoint, String)] = []
    let dt: CGFloat = 1.0 / 120
    var tt: CGFloat = 0, c = P(300, 100)
    var idx = 0, hold: CGFloat = 0
    let every = seed == 0 ? 4 : 8
    while idx < script.count {
        let (x, y, h) = script[idx]
        c = lerp(c, P(x, y), 0.06)
        hold += dt
        if hold > h { hold = 0; idx += 1 }
        cur.update(c, dt)
        r.step(dt, cur.s)
        tt += dt
        if Int((tt * 120).rounded()) % every == 0 { frames.append((r.displayPose, r.pos + r.drawOffset, r.facing, c, "\(r.mode)")) }
    }
    let cols = 10, cw: CGFloat = 150, ch: CGFloat = 130
    let shown = Array(frames.prefix(200))
    let rows = (shown.count + cols - 1) / cols
    let size = NSSize(width: CGFloat(cols) * cw, height: CGFloat(rows) * ch)
    let (rep, ctx) = makeCanvas(size, scale: 2)
    ctx.setFillColor(NSColor(white: 0.14, alpha: 1).cgColor)
    ctx.fill(CGRect(origin: .zero, size: size))
    for (i, f) in shown.enumerated() {
        let col = i % cols, row = rows - 1 - i / cols
        let cell = CGRect(x: CGFloat(col) * cw, y: CGFloat(row) * ch, width: cw, height: ch)
        ctx.saveGState()
        ctx.clip(to: cell.insetBy(dx: 1, dy: 1))
        let off = P(cell.midX - f.1.x, cell.minY + 40 - f.1.y)
        ctx.setStrokeColor(NSColor(white: 0.45, alpha: 1).cgColor); ctx.setLineWidth(1)
        for (_, wr) in wins { ctx.stroke(wr.offsetBy(dx: off.x, dy: off.y)) }
        ctx.setStrokeColor(NSColor.systemGreen.withAlphaComponent(0.6).cgColor)
        for s in w.segs { ctx.move(to: P(s.x0, s.y) + off); ctx.addLine(to: P(s.x1, s.y) + off) }
        ctx.strokePath()
        ctx.setFillColor(NSColor.systemRed.cgColor)
        let cp = f.3 + off
        ctx.fillEllipse(in: CGRect(x: cp.x - 2.5, y: cp.y - 2.5, width: 5, height: 5))
        drawFigure(ctx, f.0, f.1 + off, f.2)
        ("\(i) \(f.4)" as NSString).draw(at: NSPoint(x: cell.minX + 3, y: cell.maxY - 12),
                                         withAttributes: [.font: NSFont.systemFont(ofSize: 8), .foregroundColor: NSColor(white: 0.75, alpha: 1)])
        ctx.restoreGState()
    }
    saveCanvas(rep, path)
    print("\(frames.count) frames")
}

func selfTest() {
    _ = NSApplication.shared
    let w = World()
    w.refresh(force: true)
    print(String(format: "screens=%d windows=%d segs=%d walls=%d nodes=%d edges=%d build=%.1fms",
                 w.screens.count, w.rects.count, w.segs.count, w.walls.count, w.nodes.count, w.edgeCount, w.buildMs))
    guard let floor = w.segs.indices.first(where: { w.segs[$0].owner < 0 && w.segs[$0].owner > -100 }) else { return }
    let s = w.segs[floor]
    let vis = w.screens[0].visible
    var worst = 0.0
    for _ in 0..<300 {
        let c = P(CGFloat.random(in: vis.minX...vis.maxX), CGFloat.random(in: vis.minY...vis.maxY))
        let t0 = CACurrentMediaTime()
        _ = w.plan(seg: floor, x: (s.x0 + s.x1) / 2, cursor: c, prevSig: 0, climbMult: 1)
        worst = max(worst, (CACurrentMediaTime() - t0) * 1000)
    }
    print(String(format: "300 random plans from the floor, worst %.2fms", worst))
    let r = Runner(world: w, cursor: P(vis.midX, vis.midY))
    let cur = CursorTracker()
    var c = P(vis.midX, vis.midY), target = c
    var outside = 0
    var modes: [String: Int] = [:]
    let dt: CGFloat = 1.0 / 120
    for i in 0..<(120 * 480) {
        if i % 240 == 0 { target = P(CGFloat.random(in: vis.minX...vis.maxX), CGFloat.random(in: vis.minY...vis.maxY)) }
        c = lerp(c, target, 0.02)
        cur.update(c, dt)
        r.step(dt, cur.s)
        modes["\(r.mode)", default: 0] += 1
        if r.mode != .hang && r.mode != .ride && (r.pos.x < vis.minX - 1 || r.pos.x > vis.maxX + 1 || r.pos.y < vis.minY - 1 || r.pos.y > w.screens[0].frame.maxY) {
            outside += 1
        }
    }
    print("simulated 480s: frames off-screen=\(outside)", modes.sorted { $0.value > $1.value })
}

/// Hangs him on a cursor, then moves the cursor in circles and reports how he swings.
func spinTest() {
    _ = NSApplication.shared
    let w = World()
    let frame = CGRect(x: 0, y: 0, width: 1470, height: 956)
    w.screens = [(frame: frame, visible: frame)]
    w.rebuild([])
    let dt: CGFloat = 1.0 / 240
    for (radius, hz) in [(CGFloat(0), CGFloat(0)), (15, 1.0), (25, 1.1), (40, 1.1), (40, 1.4), (70, 1.2), (100, 1.0), (60, 2.2)] {
        let r = Runner(world: w, cursor: P(700, 600))
        let cur = CursorTracker()
        let centre = P(700, 600)
        cur.update(centre, dt)
        r.startHang(cur.s)
        var loops: CGFloat = 0, lastTheta = r.theta, released: CGFloat = -1, maxAngle: CGFloat = 0
        var tt: CGFloat = 0
        while tt < 8 {
            tt += dt
            let ramp = min(tt / 1.5, 1)
            let c = centre + P(cos(2 * .pi * hz * tt) - 1, sin(2 * .pi * hz * tt)) * (radius * ramp)
            cur.update(c, dt)
            r.step(dt, cur.s)
            if r.mode != .hang { released = tt; break }
            loops += angleDiff(lastTheta, r.theta) / (2 * .pi)
            lastTheta = r.theta
            maxAngle = max(maxAngle, abs(r.theta))
        }
        print(String(format: "circle r=%3.0fpt %.1fHz: net %.1f rev, max angle %3.0f°, %@", radius, hz, abs(loops), maxAngle * 180 / .pi,
                     released >= 0 ? String(format: "let go at %.1fs (dizzy=%@)", released, r.dizzy ? "yes" : "no") : "still holding"))
    }
}

// MARK: - Main

let args = CommandLine.arguments
if let i = args.firstIndex(of: "--snapshot"), i + 1 < args.count { renderSnapshot(to: args[i + 1]); exit(0) }
if args.contains("--selftest") { selfTest(); exit(0) }
if args.contains("--spin-test") { spinTest(); exit(0) }
if let i = args.firstIndex(of: "--film"), i + 1 < args.count { renderFilm(to: args[i + 1], seed: args.contains("--alt") ? 1 : 0); exit(0) }
if let i = args.firstIndex(of: "--vision-test"), i + 2 < args.count { visionTest(args[i + 1], args[i + 2]); exit(0) }

let app = NSApplication.shared
let delegate = AppDelegate()
app.delegate = delegate
app.run()

import * as Utils from '../utils/utils'
import * as Intersection from '../algorithms/intersection'
import * as g from '../classes'
import { PlanarSet } from '../data_structures/PlanarSet'
import { IntervalTree } from '../data_structures/interval-tree'
import { bezierClosestPoint, bezierDerivative, bezierPoint } from './bezierAlgebraicAlgoritms'

export function reverse(result: [number, g.Segment]): [number, g.Segment] {
  result[1] = result[1].reverse()
  return result
}

/**
 * Calculate distance and shortest segment between points
 */
export function point2point(a: g.Point, b: g.Point): [number, g.Segment] {
  return a.distanceTo(b)
}

/**
 * Calculate distance and shortest segment between point and line
 */
export function point2line(pt: g.Point, line: g.Line): [number, g.Segment] {
  let closest_point = pt.projectionOn(line)
  let vec = new g.Vector(pt, closest_point)
  return [vec.length, new g.Segment(pt, closest_point)]
}

/**
 * Calculate distance and shortest segment between point and circle
 */
export function point2circle(pt: g.Point, circle: g.Circle): [number, g.Segment] {
  let [dist2center, shortest_dist] = pt.distanceTo(circle.center)
  if (Utils.EQ_0(dist2center)) {
    return [circle.r, new g.Segment(pt, circle.toArc().start)]
  } else {
    let dist = Math.abs(dist2center - circle.r)
    let v = new g.Vector(circle.pc, pt).normalize().multiply(circle.r)
    let closest_point = circle.pc.translate(v)
    return [dist, new g.Segment(pt, closest_point)]
  }
}

/**
 * Calculate distance and shortest segment between point and segment
 */
export function point2segment(pt: g.Point, segment: g.Segment): [number, g.Segment] {
  /* Degenerated case of zero-length segment */
  if (segment.start.equalTo(segment.end)) {
    return point2point(pt, segment.start)
  }

  let v_seg = new g.Vector(segment.start, segment.end)
  let v_ps2pt = new g.Vector(segment.start, pt)
  let v_pe2pt = new g.Vector(segment.end, pt)
  let start_sp = v_seg.dot(v_ps2pt)
  /* dot product v_seg * v_ps2pt */
  let end_sp = -v_seg.dot(v_pe2pt)
  /* minus dot product v_seg * v_pe2pt */

  let dist
  let closest_point
  if (Utils.GE(start_sp, 0) && Utils.GE(end_sp, 0)) {
    /* point inside segment scope */
    let v_unit = segment.tangentInStart() // new g.Vector(v_seg.x / this.length, v_seg.y / this.length);
    /* unit vector ||v_unit|| = 1 */
    dist = Math.abs(v_unit.cross(v_ps2pt))
    /* dist = abs(v_unit x v_ps2pt) */
    closest_point = segment.start.translate(v_unit.multiply(v_unit.dot(v_ps2pt)))
    return [dist, new g.Segment(pt, closest_point)]
  } else if (start_sp < 0) {
    /* point is out of scope closer to start */
    return pt.distanceTo(segment.start)
  } else {
    /* point is out of scope closer to end */
    return pt.distanceTo(segment.end)
  }
}

/**
 * Calculate distance and shortest segment between point and arc
 */
export function point2arc(pt: g.Point, arc: g.Arc): [number, g.Segment] {
  let circle = new g.Circle(arc.pc, arc.r)
  let dist_and_segment = []
  let dist, shortest_segment
  ;[dist, shortest_segment] = point2circle(pt, circle)
  if (shortest_segment.end.on(arc)) {
    dist_and_segment.push(point2circle(pt, circle))
  }
  dist_and_segment.push(point2point(pt, arc.start))
  dist_and_segment.push(point2point(pt, arc.end))

  sort(dist_and_segment)

  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between segment and point
 */
export function segment2point(segment: g.Segment, point: g.Point): [number, g.Segment] {
  const result = point2segment(point, segment)
  result[1] = result[1].reverse()
  return result
}

/**
 * Calculate distance and shortest segment between segment and quadratic
 */
export function segment2quadratic(segment: g.Segment, quadratic: g.Quadratic): [number, g.Segment] {
  // Case 1: Segment and quadratic intersect - return zero distance
  let ip = Intersection.intersectSegment2Quadratic(segment, quadratic)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // Case 2: No intersection - find minimum distance
  // Check distances from segment endpoints to quadratic and vice versa
  let dist_and_segment: [number, g.Segment][] = []

  // Distance from segment start to quadratic
  let [dist_start, seg_start] = quadratic.distanceToPoint(segment.start)
  dist_and_segment.push([dist_start, seg_start.reverse()])

  // Distance from segment end to quadratic
  let [dist_end, seg_end] = quadratic.distanceToPoint(segment.end)
  dist_and_segment.push([dist_end, seg_end.reverse()])

  // Distance from quadratic start to segment
  dist_and_segment.push(point2segment(quadratic.start, segment))

  // Distance from quadratic end to segment
  dist_and_segment.push(point2segment(quadratic.end, segment))

  // Check distances from sample points along the quadratic curve to segment
  const sampleCount = 10
  for (let i = 1; i < sampleCount; i++) {
    const t = i / sampleCount
    const point = quadratic.pointAtLength(quadratic.length * t)
    dist_and_segment.push(point2segment(point, segment))
  }

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between segment and bezier
 */
export function segment2bezier(segment: g.Segment, bezier: g.Bezier): [number, g.Segment] {
  // Case 1: Segment and bezier intersect - return zero distance
  let ip = Intersection.intersectSegment2Bezier(segment, bezier)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // Case 2: No intersection - find minimum distance
  // Check distances from segment endpoints to bezier and vice versa
  let dist_and_segment: [number, g.Segment][] = []

  // Distance from segment start to bezier
  let [dist_start, seg_start] = bezier.distanceToPoint(segment.start)
  dist_and_segment.push([dist_start, seg_start.reverse()])

  // Distance from segment end to bezier
  let [dist_end, seg_end] = bezier.distanceToPoint(segment.end)
  dist_and_segment.push([dist_end, seg_end.reverse()])

  // Distance from bezier start to segment
  dist_and_segment.push(point2segment(bezier.start, segment))

  // Distance from bezier end to segment
  dist_and_segment.push(point2segment(bezier.end, segment))

  // Check distances from sample points along the bezier curve to segment
  const sampleCount = 10
  for (let i = 1; i < sampleCount; i++) {
    const t = i / sampleCount
    const point = bezier.pointAtLength(bezier.length * t)
    dist_and_segment.push(point2segment(point, segment))
  }

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between segment and polygon
 */
export function segment2polygon(segment: g.Segment, polygon: g.Polygon): [number, g.Segment] {
  let min_dist_and_segment = [Number.POSITIVE_INFINITY, new g.Segment()] as [number, g.Segment]
  for (let edge of polygon.edges) {
    let [dist, shortest_segment] = segment.distanceTo(edge.shape)
    if (Utils.LT(dist, min_dist_and_segment[0])) {
      min_dist_and_segment = [dist, shortest_segment] as [number, g.Segment]
    }
  }
  return min_dist_and_segment
}

/**
 * Calculate distance and shortest segment between segment and line
 */
export function segment2line(seg: g.Segment, line: g.Line): [number, g.Segment] {
  let ip = seg.intersect(line)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])] // distance = 0, closest point is the first point
  }
  let dist_and_segment = []
  dist_and_segment.push(point2line(seg.start, line))
  dist_and_segment.push(point2line(seg.end, line))

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between two segments
 * @param seg1
 * @param seg2
 * @returns {Number | Segment} - distance and shortest segment
 */
export function segment2segment(seg1, seg2): [number, g.Segment] {
  let ip = Intersection.intersectSegment2Segment(seg1, seg2)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])] // distance = 0, closest point is the first point
  }

  // Seg1 and seg2 not intersected
  let dist_and_segment = []
  let dist_tmp, shortest_segment_tmp
  ;[dist_tmp, shortest_segment_tmp] = point2segment(seg2.start, seg1)
  dist_and_segment.push([dist_tmp, shortest_segment_tmp.reverse()])
  ;[dist_tmp, shortest_segment_tmp] = point2segment(seg2.end, seg1)
  dist_and_segment.push([dist_tmp, shortest_segment_tmp.reverse()])
  dist_and_segment.push(point2segment(seg1.start, seg2))
  dist_and_segment.push(point2segment(seg1.end, seg2))

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between segment and circle
 * @param seg
 * @param circle
 * @returns {Number | Segment} - distance and shortest segment
 */
export function segment2circle(seg: g.Segment, circle: g.Circle): [number, g.Segment] {
  /* Case 1 Segment and circle intersected. Return the first point and zero distance */
  let ip = seg.intersect(circle)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // No intersection between segment and circle

  /* Case 2. Distance to projection of center point to line bigger than radius
   * And projection point belong to segment
   * Then measure again distance from projection to circle and return it */
  let line = new g.Line(seg.start, seg.end)
  let [dist, shortest_segment] = point2line(circle.center, line)
  if (Utils.GE(dist, circle.r) && shortest_segment.end.on(seg)) {
    return point2circle(shortest_segment.end, circle)
  } else {
  /* Case 3. Otherwise closest point is one of the end points of the segment */
    let [dist_from_start, shortest_segment_from_start] = point2circle(seg.start, circle)
    let [dist_from_end, shortest_segment_from_end] = point2circle(seg.end, circle)
    return Utils.LT(dist_from_start, dist_from_end)
      ? [dist_from_start, shortest_segment_from_start]
      : [dist_from_end, shortest_segment_from_end]
  }
}

/**
 * Calculate distance and shortest segment between segment and arc
 * @param seg
 * @param arc
 * @returns {Number | Segment} - distance and shortest segment
 */
export function segment2arc(seg, arc): [number, g.Segment] {
  /* Case 1 Segment and arc intersected. Return the first point and zero distance */
  let ip = seg.intersect(arc)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // No intersection between segment and arc
  let line = new g.Line(seg.start, seg.end)
  let circle = new g.Circle(arc.pc, arc.r)

  /* Case 2. Distance to projection of center point to line bigger than radius AND
   * projection point belongs to segment AND
   * distance from projection point to circle belongs to arc  =>
   * return this distance from projection to circle */
  let [dist_from_center, shortest_segment_from_center] = point2line(circle.center, line)
  if (Utils.GE(dist_from_center, circle.r) && shortest_segment_from_center.end.on(seg)) {
    let [dist_from_projection, shortest_segment_from_projection] = point2circle(
      shortest_segment_from_center.end,
      circle,
    )
    if (shortest_segment_from_projection.end.on(arc)) {
      return [dist_from_projection, shortest_segment_from_projection]
    }
  }
  /* Case 3. Otherwise closest point is one of the end points of the segment */
  let dist_and_segment = []
  dist_and_segment.push(point2arc(seg.start, arc))
  dist_and_segment.push(point2arc(seg.end, arc))

  let dist_tmp, segment_tmp
  ;[dist_tmp, segment_tmp] = point2segment(arc.start, seg)
  dist_and_segment.push([dist_tmp, segment_tmp.reverse()])

  ;[dist_tmp, segment_tmp] = point2segment(arc.end, seg)
  dist_and_segment.push([dist_tmp, segment_tmp.reverse()])

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between two circles
 * @param circle1
 * @param circle2
 * @returns {Number | Segment} - distance and shortest segment
 */
export function circle2circle(circle1, circle2): [number, g.Segment] {
  let ip = circle1.intersect(circle2)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // Case 1. Concentric circles. Convert to arcs and take distance between two arc starts
  if (circle1.center.equalTo(circle2.center)) {
    let arc1 = circle1.toArc()
    let arc2 = circle2.toArc()
    return point2point(arc1.start, arc2.start)
  } else {
    // Case 2. Not concentric circles
    let line = new g.Line(circle1.center, circle2.center)
    let ip1 = line.intersect(circle1)
    let ip2 = line.intersect(circle2)

    let dist_and_segment = []

    dist_and_segment.push(point2point(ip1[0], ip2[0]))
    dist_and_segment.push(point2point(ip1[0], ip2[1]))
    dist_and_segment.push(point2point(ip1[1], ip2[0]))
    dist_and_segment.push(point2point(ip1[1], ip2[1]))

    sort(dist_and_segment)
    return dist_and_segment[0]
  }
}

/**
 * Calculate distance and shortest segment between two circles
 * @param circle
 * @param line
 * @returns {Number | Segment} - distance and shortest segment
 */
export function circle2line(circle, line): [number, g.Segment] {
  let ip = circle.intersect(line)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  let [dist_from_center, shortest_segment_from_center] = point2line(circle.center, line)
  let [dist, shortest_segment] = point2circle(shortest_segment_from_center.end, circle)
  shortest_segment = shortest_segment.reverse()
  return [dist, shortest_segment]
}

/**
 * Calculate distance and shortest segment between an arc and a point
 */
export function arc2point(arc: g.Arc, point: g.Point) {
  return reverse(point2arc(point, arc))
}

/**
 * Calculate distance and shortest segment between an arc and a point
 */
export function arc2segment(arc: g.Arc, segment: g.Segment) {
  return reverse(segment2arc(segment, arc))
}

/**
 * Calculate distance and shortest segment between arc and line
 * @param arc
 * @param line
 * @returns {Number | Segment} - distance and shortest segment
 */
export function arc2line(arc, line): [number, g.Segment] {
  /* Case 1 Line and arc intersected. Return the first point and zero distance */
  let ip = line.intersect(arc)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  let circle = new g.Circle(arc.center, arc.r)

  /* Case 2. Distance to projection of center point to line bigger than radius AND
   * projection point belongs to segment AND
   * distance from projection point to circle belongs to arc  =>
   * return this distance from projection to circle */
  let [dist_from_center, shortest_segment_from_center] = point2line(circle.center, line)
  if (Utils.GE(dist_from_center, circle.r)) {
    let [dist_from_projection, shortest_segment_from_projection] = point2circle(
      shortest_segment_from_center.end,
      circle,
    )
    if (shortest_segment_from_projection.end.on(arc)) {
      return [dist_from_projection, shortest_segment_from_projection]
    }
  } else {
    let dist_and_segment = []
    dist_and_segment.push(point2line(arc.start, line))
    dist_and_segment.push(point2line(arc.end, line))

    sort(dist_and_segment)
    return dist_and_segment[0]
  }
}

/**
 * Calculate distance and shortest segment between arc and circle
 * @param arc
 * @param circle2
 * @returns {Number | Segment} - distance and shortest segment
 */
export function arc2circle(arc, circle2): [number, g.Segment] {
  let ip = arc.intersect(circle2)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  let circle1 = new g.Circle(arc.center, arc.r)

  let [dist, shortest_segment] = circle2circle(circle1, circle2)
  if (shortest_segment.start.on(arc)) {
    return [dist, shortest_segment]
  } else {
    let dist_and_segment = []

    dist_and_segment.push(point2circle(arc.start, circle2))
    dist_and_segment.push(point2circle(arc.end, circle2))

    sort(dist_and_segment)

    return dist_and_segment[0]
  }
}

/**
 * Calculate distance and shortest segment between quadratic and circle
 */
export function quadratic2circle(quadratic: g.Quadratic, circle: g.Circle): [number, g.Segment] {
  // Case 1: Quadratic and circle intersect - return zero distance
  let ip = Intersection.intersectCircle2Quadratic(circle, quadratic)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // Case 2: No intersection - find minimum distance
  let dist_and_segment: [number, g.Segment][] = []

  // Distance from quadratic start to circle
  dist_and_segment.push(point2circle(quadratic.start, circle))

  // Distance from quadratic end to circle
  dist_and_segment.push(point2circle(quadratic.end, circle))

  // Check distances from sample points along the quadratic curve to circle
  const sampleCount = 10
  for (let i = 1; i < sampleCount; i++) {
    const t = i / sampleCount
    const point = quadratic.pointAtLength(quadratic.length * t)
    dist_and_segment.push(point2circle(point, circle))
  }

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Calculate distance and shortest segment between bezier and circle
 */
export function bezier2circle(bezier: g.Bezier, circle: g.Circle): [number, g.Segment] {
  // Case 1: Bezier and circle intersect - return zero distance
  let ip = Intersection.intersectCircle2Bezier(circle, bezier)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  // Case 2: No intersection - find minimum distance
  let dist_and_segment: [number, g.Segment][] = []

  // Distance from bezier start to circle
  dist_and_segment.push(point2circle(bezier.start, circle))

  // Distance from bezier end to circle
  dist_and_segment.push(point2circle(bezier.end, circle))

  // Check distances from sample points along the bezier curve to circle
  const sampleCount = 10
  for (let i = 1; i < sampleCount; i++) {
    const t = i / sampleCount
    const point = bezier.pointAtLength(bezier.length * t)
    dist_and_segment.push(point2circle(point, circle))
  }

  sort(dist_and_segment)
  return dist_and_segment[0]
}

/**
 * Find closest points between two cubic Bezier curves using algebraic/numerical method
 * 
 * Mathematical approach:
 * Minimize D²(t,u) = ||B₁(t) - B₂(u)||²
 * Critical points satisfy:
 *   (B₁(t) - B₂(u)) · B₁'(t) = 0
 *   (B₁(t) - B₂(u)) · B₂'(u) = 0
 * 
 * This system is solved numerically with Newton-Raphson method.
 * 
 * @returns [minimum distance, segment connecting closest points]
 */
export function bezier2bezier(bezier1: g.Bezier, bezier2: g.Bezier): [number, g.Segment] {
  // Extract control points
    const p = [bezier1.start, bezier1.control1, bezier1.control2, bezier1.end]
    const q = [bezier2.start, bezier2.control1, bezier2.control2, bezier2.end]
    
    // Helper: Euclidean distance between points
    const pointDist = (a: g.Point, b: g.Point): number => 
      Math.hypot(a.x - b.x, a.y - b.y)
    
    // Helper: update minimum if candidate is better
    let minDist = Infinity
    let minSeg = new g.Segment()
    
    const updateMin = (pt1: g.Point, pt2: g.Point) => {
      const dist = pointDist(pt1, pt2)
      if (dist < minDist) {
        minDist = dist
        minSeg = new g.Segment(pt1, pt2)
      }
    }
    
    // ============================================================================
    // STEP 1: Check for intersections (distance = 0)
    // ============================================================================
    try {
      const intersections = bezier1.intersect(bezier2)
      if (intersections && intersections.length > 0) {
        return [0, new g.Segment(intersections[0], intersections[0])]
      }
    } catch (e) {
      // If intersect() not implemented or fails, continue with distance calculation
    }
    
    // ============================================================================
    // STEP 2: Check all control point combinations (quick bounds)
    // ============================================================================
    for (const pt1 of p) {
      for (const pt2 of q) {
        updateMin(pt1, pt2)
      }
    }
    
    // ============================================================================
    // STEP 3: Check endpoints of curve1 against full curve2 (and vice versa)
    // ============================================================================
    for (const pt of p) {
      const [dist, seg] = bezierClosestPoint(bezier2, pt)
      if (dist < minDist) {
        minDist = dist
        minSeg = seg.reverse() // seg goes from curve2 to pt, we need pt to curve2
      }
    }
    
    for (const pt of q) {
      const [dist, seg] = bezierClosestPoint(bezier1, pt)
      if (dist < minDist) {
        minDist = dist
        minSeg = seg // seg already goes from curve1 to pt
      }
    }
    
    // ============================================================================
    // STEP 4: Numerical optimization - solve the critical point system
    // 
    // We minimize F(t,u) = ||B₁(t) - B₂(u)||²
    // Critical points: ∂F/∂t = 0 and ∂F/∂u = 0
    // Which gives:
    //   f(t,u) = (B₁(t) - B₂(u)) · B₁'(t) = 0
    //   g(t,u) = (B₁(t) - B₂(u)) · B₂'(u) = 0  (note: sign doesn't matter for root-finding)
    //
    // We use Newton-Raphson with grid-search initialization and gradient descent fallback.
    // ============================================================================
    
    const newtonSolve = (t0: number, u0: number): [number, number] | null => {
      let t = t0, u = u0
      const maxIter = 30
      const tol = 1e-10
      const eps = 1e-6
      
      for (let iter = 0; iter < maxIter; iter++) {
        // Evaluate curves and derivatives
        const B1 = bezierPoint(p[0], p[1], p[2], p[3], t)
        const B1p = bezierDerivative(p[0], p[1], p[2], p[3], t)
        const B2 = bezierPoint(q[0], q[1], q[2], q[3], u)
        const B2p = bezierDerivative(q[0], q[1], q[2], q[3], u)
        
        const diff = new g.Vector(B1.x - B2.x, B1.y - B2.y)
        
        // System functions
        const f = diff.dot(B1p)  // (B1-B2)·B1'
        const f1 = diff.dot(B2p)  // (B1-B2)·B2'
        
        // Check convergence
        if (Math.hypot(f, f1) < tol) {
          return [Math.max(0, Math.min(1, t)), Math.max(0, Math.min(1, u))]
        }
        
        // Compute second derivatives numerically (central difference)
        const tPlus = Math.min(1, t + eps), tMinus = Math.max(0, t - eps)
        const uPlus = Math.min(1, u + eps), uMinus = Math.max(0, u - eps)
        
        const B1p_plus = bezierDerivative(p[0], p[1], p[2], p[3], tPlus)
        const B1p_minus = bezierDerivative(p[0], p[1], p[2], p[3], tMinus)
        const B2p_plus = bezierDerivative(q[0], q[1], q[2], q[3], uPlus)
        const B2p_minus = bezierDerivative(q[0], q[1], q[2], q[3], uMinus)
        
        const B1pp = new g.Vector(
          (B1p_plus.x - B1p_minus.x) / (tPlus - tMinus),
          (B1p_plus.y - B1p_minus.y) / (tPlus - tMinus)
        )
        const B2pp = new g.Vector(
          (B2p_plus.x - B2p_minus.x) / (uPlus - uMinus),
          (B2p_plus.y - B2p_minus.y) / (uPlus - uMinus)
        )
        
        // Jacobian matrix:
        // J = [ ∂f/∂t  ∂f/∂u ] = [ B1'·B1' + (B1-B2)·B1''    -B2'·B1' ]
        //     [ ∂g/∂t  ∂g/∂u ]   [ B1'·B2'                    -B2'·B2' - (B1-B2)·B2'' ]
        
        const J11 = B1p.dot(B1p) + diff.dot(B1pp)
        const J12 = -B2p.dot(B1p)
        const J21 = B1p.dot(B2p)
        const J22 = -B2p.dot(B2p) - diff.dot(B2pp)
        
        const det = J11 * J22 - J12 * J21
        
        if (Math.abs(det) < 1e-12) {
          // Singular Jacobian: fallback to gradient descent step
          const step = 0.02
          t = Math.max(0, Math.min(1, t - step * f))
          u = Math.max(0, Math.min(1, u - step * f1))
          continue
        }
        
        // Newton step: solve J * [dt, du]ᵀ = -[f, g]ᵀ
        const dt = (J12 * f1 - J22 * f) / det
        const du = (J21 * f - J11 * f1) / det
        
        // Check step convergence
        if (Math.hypot(dt, du) < tol) {
          t = Math.max(0, Math.min(1, t + dt))
          u = Math.max(0, Math.min(1, u + du))
          break
        }
        
        // Update with clamping to [0,1]
        t = Math.max(0, Math.min(1, t + dt))
        u = Math.max(0, Math.min(1, u + du))
      }
      
      return [t, u]
    }
    
    // Grid search for initial guesses (5×5 = 25 starting points)
    const gridSteps = 5
    for (let i = 0; i <= gridSteps; i++) {
      for (let j = 0; j <= gridSteps; j++) {
        const t0 = i / gridSteps
        const u0 = j / gridSteps
        
        const result = newtonSolve(t0, u0)
        if (result) {
          const [t, u] = result
          const pt1 = bezierPoint(p[0], p[1], p[2], p[3], t)
          const pt2 = bezierPoint(q[0], q[1], q[2], q[3], u)
          updateMin(pt1, pt2)
        }
      }
    }
    
    // ============================================================================
    // STEP 5: Additional sampling as safety net (catches edge cases)
    // ============================================================================
    const sampleCount = 40
    for (let i = 0; i <= sampleCount; i++) {
      const t = i / sampleCount
      const pt1 = bezierPoint(p[0], p[1], p[2], p[3], t)
      const [dist, seg] = bezierClosestPoint(bezier2, pt1)
      if (dist < minDist) {
        minDist = dist
        minSeg = seg.reverse()
      }
    }
    
    // Also sample the other direction for symmetry
    for (let j = 0; j <= sampleCount; j++) {
      const u = j / sampleCount
      const pt2 = bezierPoint(q[0], q[1], q[2], q[3], u)
      const [dist, seg] = bezierClosestPoint(bezier1, pt2)
      if (dist < minDist) {
        minDist = dist
        minSeg = seg
      }
    }
    
    return [minDist, minSeg]
}

/**
 * Find closest point between Bezier curve and a segment using algebraic method
 */
export function bezier2segment(bezier: g.Bezier, segment: g.Segment): [number, g.Segment] {
  // Check if they intersect first
    const intersections = bezier.intersect(segment)
    if (intersections.length > 0) {
      return [0, new g.Segment(intersections[0], intersections[0])]
    }
    
    let minDist = Infinity
    let minSegment = new g.Segment()
    
    // 1. Find closest point on bezier to segment endpoints
    const [dist1, seg1] = bezierClosestPoint(bezier, segment.start)
    if (dist1 < minDist) {
      minDist = dist1
      minSegment = seg1.reverse()
    }
    
    const [dist2, seg2] = bezierClosestPoint(bezier, segment.end)
    if (dist2 < minDist) {
      minDist = dist2
      minSegment = seg2.reverse()
    }
    
    // 2. Find closest point on segment to bezier endpoints
    const [dist3, seg3] = segment.distanceTo(bezier.start)
    if (dist3 < minDist) {
      minDist = dist3
      minSegment = seg3
    }
    
    const [dist4, seg4] = segment.distanceTo(bezier.end)
    if (dist4 < minDist) {
      minDist = dist4
      minSegment = seg4
    }
    
    // 3. Sample points along the bezier curve for additional checks
    const sampleCount = 20
    for (let i = 1; i < sampleCount; i++) {
      const t = i / sampleCount
      const pt = bezierPoint(bezier.start, bezier.control1, bezier.control2, bezier.end, t)
      const [dist, seg] = segment.distanceTo(pt)
      if (dist < minDist) {
        minDist = dist
        minSegment = seg
      }
    }
    
    return [minDist, minSegment]
}

/**
 * Find closest point between Bezier curve and polygon using algebraic method
 */
export function bezier2polygon(bezier: g.Bezier, polygon: g.Polygon): [number, g.Segment] {
  let minDist = Infinity
    let minSegment = new g.Segment()
    
    for (const edge of polygon.edges) {
      let dist: number
      let seg: g.Segment
      
      if (edge.isSegment()) {
        [dist, seg] = bezier2segment(bezier, edge.shape as g.Segment)
      } else if (edge.isBezier()) {
        // Bezier to Bezier - use sampling for now
        [dist, seg] = bezier.distanceTo(edge.shape)
      } else {
        // Arc or other shapes
        [dist, seg] = bezier.distanceTo(edge.shape)
      }
      
      if (dist < minDist) {
        minDist = dist
        minSegment = seg
      }
    }
    
    return [minDist, minSegment]
}

/**
 * Calculate distance and shortest segment between two arcs
 * @param arc1
 * @param arc2
 * @returns {Number | Segment} - distance and shortest segment
 */
export function arc2arc(arc1, arc2): [number, g.Segment] {
  let ip = arc1.intersect(arc2)
  if (ip.length > 0) {
    return [0, new g.Segment(ip[0], ip[0])]
  }

  let circle1 = new g.Circle(arc1.center, arc1.r)
  let circle2 = new g.Circle(arc2.center, arc2.r)

  let [dist, shortest_segment] = circle2circle(circle1, circle2)
  if (shortest_segment.start.on(arc1) && shortest_segment.end.on(arc2)) {
    return [dist, shortest_segment]
  } else {
    let dist_and_segment = []

    let dist_tmp, segment_tmp

    ;[dist_tmp, segment_tmp] = point2arc(arc1.start, arc2)
    if (segment_tmp.end.on(arc2)) {
      dist_and_segment.push([dist_tmp, segment_tmp])
    }

    ;[dist_tmp, segment_tmp] = point2arc(arc1.end, arc2)
    if (segment_tmp.end.on(arc2)) {
      dist_and_segment.push([dist_tmp, segment_tmp])
    }

    ;[dist_tmp, segment_tmp] = point2arc(arc2.start, arc1)
    if (segment_tmp.end.on(arc1)) {
      dist_and_segment.push([dist_tmp, segment_tmp.reverse()])
    }

    ;[dist_tmp, segment_tmp] = point2arc(arc2.end, arc1)
    if (segment_tmp.end.on(arc1)) {
      dist_and_segment.push([dist_tmp, segment_tmp.reverse()])
    }

    ;[dist_tmp, segment_tmp] = point2point(arc1.start, arc2.start)
    dist_and_segment.push([dist_tmp, segment_tmp])

    ;[dist_tmp, segment_tmp] = point2point(arc1.start, arc2.end)
    dist_and_segment.push([dist_tmp, segment_tmp])

    ;[dist_tmp, segment_tmp] = point2point(arc1.end, arc2.start)
    dist_and_segment.push([dist_tmp, segment_tmp])

    ;[dist_tmp, segment_tmp] = point2point(arc1.end, arc2.end)
    dist_and_segment.push([dist_tmp, segment_tmp])

    sort(dist_and_segment)

    return dist_and_segment[0]
  }
}

/**
 * Calculate distance and shortest segment between point and polygon
 * @param point
 * @param polygon
 * @returns {Number | Segment} - distance and shortest segment
 */
export function point2polygon(point, polygon): [number, g.Segment] {
  let min_dist_and_segment = [Number.POSITIVE_INFINITY, new g.Segment()] as [number, g.Segment]
  for (let edge of polygon.edges) {
    let [dist, shortest_segment] =
      edge.shape instanceof g.Segment ? point2segment(point, edge.shape) : point2arc(point, edge.shape)
    if (Utils.LT(dist, min_dist_and_segment[0])) {
      min_dist_and_segment = [dist, shortest_segment] as [number, g.Segment]
    }
  }
  return min_dist_and_segment
}

/**
 * Calculate distance and shortest segment between point and quadratic
 */
export function point2quadratic(point: g.Point, quadratic: g.Quadratic): [number, g.Segment] {
  // Check if point is on the quadratic curve
  if (quadratic.contains(point)) {
    return [0, new g.Segment(point, point)]
  }

  // Find minimum distance by checking distances to all segments of the quadratic
  let min_dist = Number.POSITIVE_INFINITY
  let closest_segment = new g.Segment(point, point)

  for (let segment of quadratic.segments) {
    let [dist, shortest_segment] = point2segment(point, segment)
    if (dist < min_dist) {
      min_dist = dist
      closest_segment = shortest_segment
    }
  }

  return [min_dist, closest_segment]
}

/**
 * Calculate distance and shortest segment between point and bezier
 */
export function point2bezier(point: g.Point, bezier: g.Bezier): [number, g.Segment] {
  // Check if point is on the bezier curve
  if (bezier.contains(point)) {
    return [0, new g.Segment(point, point)]
  }

  // Find minimum distance by checking distances to all segments of the bezier
  let min_dist = Number.POSITIVE_INFINITY
  let closest_segment = new g.Segment(point, point)

  for (let segment of bezier.segments) {
    let [dist, shortest_segment] = point2segment(point, segment)
    if (dist < min_dist) {
      min_dist = dist
      closest_segment = shortest_segment
    }
  }

  return [min_dist, closest_segment]
}

export function shape2polygon(shape, polygon): [number, g.Segment] {
  let min_dist_and_segment = [Number.POSITIVE_INFINITY, new g.Segment()] as [number, g.Segment]
  for (let edge of polygon.edges) {
    let [dist, shortest_segment] = shape.distanceTo(edge.shape)
    if (Utils.LT(dist, min_dist_and_segment[0])) {
      min_dist_and_segment = [dist, shortest_segment] as [number, g.Segment]
    }
  }
  return min_dist_and_segment
}

/**
 * Calculate distance and shortest segment between two polygons
 * @param polygon1
 * @param polygon2
 * @returns {Number | Segment} - distance and shortest segment
 */
export function polygon2polygon(polygon1, polygon2): [number, g.Segment] {
  let min_dist_and_segment = [Number.POSITIVE_INFINITY, new g.Segment()] as [number, g.Segment]
  for (let edge1 of polygon1.edges) {
    for (let edge2 of polygon2.edges) {
      let [dist, shortest_segment] = edge1.shape.distanceTo(edge2.shape)
      if (Utils.LT(dist, min_dist_and_segment[0])) {
        min_dist_and_segment = [dist, shortest_segment] as [number, g.Segment]
      }
    }
  }
  return min_dist_and_segment
}

/**
 * Returns [mindist, maxdist] array of squared minimal and maximal distance between boxes
 * Minimal distance by x is
 *    (box2.xmin - box1.xmax), if box1 is left to box2
 *    (box1.xmin - box2.xmax), if box2 is left to box1
 *    0,                       if box1 and box2 are intersected by x
 * Minimal distance by y is defined in the same way
 *
 * Maximal distance is estimated as a sum of squared dimensions of the merged box
 *
 * @param box1
 * @param box2
 */
export function box2box_minmax(box1, box2) {
  let mindist_x = Math.max(Math.max(box1.xmin - box2.xmax, 0), Math.max(box2.xmin - box1.xmax, 0))
  let mindist_y = Math.max(Math.max(box1.ymin - box2.ymax, 0), Math.max(box2.ymin - box1.ymax, 0))
  let mindist = mindist_x * mindist_x + mindist_y * mindist_y

  let box = box1.merge(box2)
  let dx = box.xmax - box.xmin
  let dy = box.ymax - box.ymin
  let maxdist = dx * dx + dy * dy

  return [mindist, maxdist] as const
}

export function minmax_tree_process_level(shape, level, min_stop, tree) {
  // Calculate minmax distance to each shape in current level
  // Insert result into the interval tree for further processing
  // update min_stop with maxdist, it will be the new stop distance
  let mindist, maxdist
  for (let node of level) {
    // [mindist, maxdist] = box2box_minmax(shape.box, node.max);
    // if (Utils.GT(mindist, min_stop))
    //     continue;

    // Estimate min-max dist to the shape stored in the node.item, using node.item.key which is shape's box
    ;[mindist, maxdist] = box2box_minmax(shape.box, node.item.key)
    if (node.item.value instanceof g.Edge) {
      tree.insert([mindist, maxdist], node.item.value.shape)
    } else {
      tree.insert([mindist, maxdist], node.item.value)
    }
    if (Utils.LT(maxdist, min_stop)) {
      min_stop = maxdist // this will be the new distance estimation
    }
  }

  if (level.length === 0) return min_stop

  // Calculate new level from left and right children of the current
  let new_level_left = level
    .map((node) => (node.left.isNil() ? undefined : node.left))
    .filter((node) => node !== undefined)
  let new_level_right = level
    .map((node) => (node.right.isNil() ? undefined : node.right))
    .filter((node) => node !== undefined)
  // Merge left and right subtrees and leave only relevant subtrees
  let new_level = [...new_level_left, ...new_level_right].filter((node) => {
    // Node subtree quick reject, node.max is a subtree box
    let [mindist, maxdist] = box2box_minmax(shape.box, node.max)
    return Utils.LE(mindist, min_stop)
  })

  min_stop = minmax_tree_process_level(shape, new_level, min_stop, tree)
  return min_stop
}

/**
 * Calculates sorted tree of [mindist, maxdist] intervals between query shape
 * and shapes of the planar set.
 * @param shape
 * @param set
 */
export function minmax_tree(shape, set, min_stop) {
  let tree = new IntervalTree()
  let level = [set.index.root]
  let squared_min_stop = min_stop < Number.POSITIVE_INFINITY ? min_stop * min_stop : Number.POSITIVE_INFINITY
  squared_min_stop = minmax_tree_process_level(shape, level, squared_min_stop, tree)
  return tree
}

export function minmax_tree_calc_distance(shape, node, min_dist_and_segment) {
  let min_dist_and_segment_new, stop
  if (node != null && !node.isNil()) {
    ;[min_dist_and_segment_new, stop] = minmax_tree_calc_distance(shape, node.left, min_dist_and_segment)

    if (stop) {
      return [min_dist_and_segment_new, stop]
    }

    if (Utils.LT(min_dist_and_segment_new[0], Math.sqrt(node.item.key.low))) {
      return [min_dist_and_segment_new, true] // stop condition
    }

    let [dist, shortest_segment] = distance(shape, node.item.value)
    // console.log(dist)
    if (Utils.LT(dist, min_dist_and_segment_new[0])) {
      min_dist_and_segment_new = [dist, shortest_segment]
    }

    ;[min_dist_and_segment_new, stop] = minmax_tree_calc_distance(shape, node.right, min_dist_and_segment_new)

    return [min_dist_and_segment_new, stop]
  }

  return [min_dist_and_segment, false]
}

/**
 * Calculates distance between shape and Planar Set of shapes
 * @param shape
 * @param {PlanarSet} set
 * @param {Number} min_stop
 */
export function shape2planarSet(shape, set, min_stop = Number.POSITIVE_INFINITY) {
  let min_dist_and_segment = [min_stop, new g.Segment()] as [number, g.Segment]
  let stop = false
  if (set instanceof PlanarSet) {
    let tree = minmax_tree(shape, set, min_stop)
    ;[min_dist_and_segment, stop] = minmax_tree_calc_distance(shape, tree.root, min_dist_and_segment)
  }
  return min_dist_and_segment
}

export function sort(dist_and_segment) {
  dist_and_segment.sort((d1, d2) => {
    if (Utils.LT(d1[0], d2[0])) {
      return -1
    }
    if (Utils.GT(d1[0], d2[0])) {
      return 1
    }
    return 0
  })
}

export function distance(shape1, shape2) {
  return shape1.distanceTo(shape2)
}

export function pythagore(width: number, height: number) {
  return Math.sqrt(width * width + height * height)
}

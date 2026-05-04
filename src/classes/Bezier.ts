import * as geom from './index'
import { Arc } from './Arc'
import { Point } from './Point'
import { Segment } from './Segment'
import { Shape, ShapeTag } from './Shape'
import { lerp } from '../utils/lerp'
import * as Utils from '../utils/utils'
import * as Intersection from '../algorithms/intersection'
import * as Distance from '../algorithms/distance'
import * as BezierAlgebraic from '../algorithms/bezierAlgebraicAlgoritms'
import * as curves from './curves'
import { Quadratic } from './Quadratic'

const EMPTY = Object.freeze([]) as any[]

/**
 * Class representing a cubic bezier
 * @type {Bezier}
 */
export class Bezier extends Shape<Bezier> {
  static EMPTY = Object.seal((() => {
    const b = new Bezier(Point.EMPTY, Point.EMPTY, Point.EMPTY, Point.EMPTY)
    b.vertices
    b.segments
    b.lut
    return b
  })())

  /** Start point */
  start: Point
  /** End point */
  end: Point
  /** Control point 1 */
  control1: Point
  /** Control point 2 */
  control2: Point

  _lut: number[]
  _vertices: Point[]
  _segments: Segment[]

  constructor(other: Bezier)
  constructor(start: Point, control1: Point, control2: Point, end: Point)
  constructor(a: unknown, b?: unknown, c?: unknown, d?: unknown) {
    super()

    if (a instanceof Bezier) {
      this.start = a.start
      this.end = a.end
      this.control1 = a.control1
      this.control2 = a.control2
    } else {
      this.start = a as any
      this.end = d as any
      this.control1 = b as any
      this.control2 = c as any
    }

    this._lut = EMPTY
    this._vertices = EMPTY
    this._segments = EMPTY
  }

  /**
   * Return new cloned instance of segment
   */
  clone() {
    return new Bezier(this.start, this.control1, this.control2, this.end)
  }

  get tag() {
    return ShapeTag.Bezier
  }

  /**
   * Returns LUT
   */
  get lut() {
    if (this._lut === EMPTY) {
      this._lut = curves.bezier.generateLUT(
        this.start.x,
        this.start.y,
        this.control1.x,
        this.control1.y,
        this.control2.x,
        this.control2.y,
        this.end.x,
        this.end.y,
      )
    }
    return this._lut
  }

  /**
   * Returns array of points
   */
  get vertices() {
    if (this._vertices === EMPTY) {
      const lut = this.lut
      this._vertices = []
      for (let i = 0; i < lut.length; i += 4) {
        const x = lut[i + 0]
        const y = lut[i + 1]
        this._vertices.push(new Point(x, y))
      }
    }
    return this._vertices
  }

  /**
   * Returns array of segments
   */
  get segments() {
    if (this._segments === EMPTY) {
      let previous = this.vertices[0]
      this._segments = this.vertices.slice(1).reduce((result, current) => {
        result.push(new Segment(previous, current))
        previous = current
        return result
      }, [] as Segment[])
    }
    return this._segments
  }

  /**
   * Length of the curve
   */
  get length() {
    return this.lut[this.lut.length - 1]
  }

  /**
   * Bounding box
   */
  get box() {
    // FIXME: use the analytical solution
    return curves.boxFromLUT(this.lut)
  }

  get center() {
    return this.pointAtLength(this.length / 2)
  }

  /**
   * Returns true if equals to query segment, false otherwise
   */
  equalTo(other: Bezier): boolean {
    return (
      this.start.equalTo(other.start) &&
      this.end.equalTo(other.end) &&
      this.control1.equalTo(other.control1) &&
      this.control2.equalTo(other.control2)
    )
  }

  /**
   * Returns true if curve contains point
   */
  contains(point: Point): boolean {
    return BezierAlgebraic.bezierContainsPoint(this, point, Utils.getTolerance())
  }

  /**
   * Returns array of intersection points between segment and other shape
   */
  intersect(shape: Shape): Point[] {
    if (shape instanceof Point) {
      return this.contains(shape) ? [shape] : []
    }

    const intersect = getSegmentIntersect(shape) as (s: Segment, o: any) => Point[]
    const segments = this.segments.map((segment) => intersect(segment, shape)).flat()

    return segments
  }

  /**
   * Calculate distance and shortest segment from segment to shape and return as array [distance, shortest segment]
   * @param {Shape} shape Shape of the one of supported types Point, Line, Circle, Segment, Arc, Polygon or Planar Set
   * @returns {number} distance from segment to shape
   * @returns {Segment} shortest segment between segment and shape (started at segment, ended at shape)
   */
  distanceTo(shape: Shape): [number, Segment] {
    // Use algebraic method for more precise distance calculation with specific shapes
    if (shape instanceof geom.Point) {
      const [dist, seg] = BezierAlgebraic.bezierClosestPoint(this, shape)
      return [dist, seg.reverse()]
    }
    
    if (shape instanceof geom.Segment) {
      return Distance.bezier2segment(this, shape)
    }
    
    if (shape instanceof geom.Polygon) {
      return Distance.bezier2polygon(this, shape)
    }

    if (shape instanceof geom.Bezier) {
      return Distance.bezier2bezier(this, shape)
    }

    if (shape instanceof geom.Circle) {
      return Distance.bezier2circle(this, shape)
    }

    // Fall back to segment-based approximation for other shapes
    const distance = getSegmentDistance(shape)
    return this.segments.reduce(
      (result, current) => {
        const currentResult = distance(current, shape)
        if (currentResult[0] < result[0]) return currentResult
        return result
      },
      [Infinity, Segment.EMPTY as Segment],
    )
  }

  tangentInStart(): geom.Vector {
    // For a cubic Bezier curve, the tangent vector at the starting point (t=0)
    // equal to the derivative: B'(0) = 3(control1 - start)
    let vec = new geom.Vector(this.start, this.control1)
    return vec.normalize()
  }

  tangentInEnd(): geom.Vector {
    // For a cubic Bezier curve, the tangent vector at the end point (t=1)
    // equal to the derivative: B'(1) = 3(end - control2)
    let vec = new geom.Vector(this.control2, this.end)
    return vec.normalize()
  }

  /**
   * Returns new curve with swapped start and end points
   */
  reverse() {
    return new Bezier(this.end, this.control2, this.control1, this.start)
  }

  /**
   * When point belongs to segment, return array of two segments split by given point,
   * if point is inside segment. Returns clone of this segment if query point is incident
   * to start or end point of the segment. Returns empty array if point does not belong to segment
   */
  split(point: Point): (Bezier | null)[] {
    // Checking whether a point belongs to a curve
    if (!this.contains(point)) {
      return []
    }

    // If the point coincides with the starting point
    if (this.start.equalTo(point)) {
      return [this.clone()]
    }

    // If the point matches the end point
    if (this.end.equalTo(point)) {
      return [this.clone()]
    }

    // Finding the parameter t for a point on the curve
    // We use LUT (lookup table) to find the nearest point
    const lut = this.lut
    let minDistance = Infinity
    let bestIndex = -1

    for (let i = 0; i < lut.length; i += 4) {
      const x = lut[i + 0]
      const y = lut[i + 1]
      const dist = Math.hypot(x - point.x, y - point.y)
      
      if (dist < minDistance) {
        minDistance = dist
        bestIndex = i / 4
      }
    }

    // If a close point is not found, return an empty array
    if (bestIndex === -1 || minDistance > Utils.getTolerance()) {
      return []
    }

    // Getting parameter t from LUT
    const t = lut[bestIndex * 4 + 2]

    // Using splitAtT method to split a curve
    return this.splitAtT(t)
  }

  splitAtLength(length: number): (Bezier | null)[] {
    if (Utils.EQ_0(length)) return [null, this.clone()]

    if (Utils.EQ(length, this.length) || Utils.GT(length, this.length)) return [this.clone(), null]

    const lut = this.lut
    const index = curves.findIndexFromLUT(lut, length)

    const ta = lut[index * 4 + 2]
    const la = lut[index * 4 + 3]

    const tb = lut[(index + 1) * 4 + 2]
    const lb = lut[(index + 1) * 4 + 3]

    const f = (length - la) / (lb - la)
    const t = lerp(ta, tb, f)

    return this.splitAtT(t)
  }

  /**
   * @param t Factor from 0.0 to 1.0
   */
  splitAtT(t: number) {
    if (Utils.EQ_0(t)) return [null, this.clone()]

    if (Utils.GE(t, 1.0)) return [this.clone(), null]

    // https://stackoverflow.com/questions/18655135/divide-bezier-curve-into-two-equal-halves
    const A = this.start
    const B = this.control1
    const C = this.control2
    const D = this.end
    const E = pointAtRatio(A, B, t)
    const F = pointAtRatio(B, C, t)
    const G = pointAtRatio(C, D, t)
    const H = pointAtRatio(E, F, t)
    const J = pointAtRatio(F, G, t)
    const K = pointAtRatio(H, J, t)

    return [new Bezier(A, E, H, K), new Bezier(K, J, G, D)]
  }

  /**
   * Return middle point of the curve
   */
  middle(): Point {
    return this.pointAtLength(this.length / 2)
  }

  /**
   * Get point at given length
   * @param length The length along the segment
   */
  pointAtLength(length: number): Point {
    if (length === 0) {
      return this.start
    }

    const segments = this.segments

    if (segments.length === 0) return Point.EMPTY

    const lut = this.lut
    const index = curves.findIndexFromLUT(lut, length)
    const lengthAtIndex = lut[index * 4 + 3]
    const lengthInSegment = length - lengthAtIndex
    const segment = segments[index]

    return segment.pointAtLength(lengthInSegment)
  }

  distanceToPoint(point: Point) {
    const [dist, seg] = BezierAlgebraic.bezierClosestPoint(this, point)
    return [dist, seg.reverse()] as [number, Segment]
  }

  definiteIntegral(ymin = 0.0) {
    // x(t), y(t) в степенном базисе:
    // p(t) = a*t^3 + b*t^2 + c*t + d, t in [0, 1]
    const x0 = this.start.x
    const x1 = this.control1.x
    const x2 = this.control2.x
    const x3 = this.end.x
    const y0 = this.start.y
    const y1 = this.control1.y
    const y2 = this.control2.y
    const y3 = this.end.y

    const ax = -x0 + 3 * x1 - 3 * x2 + x3
    const bx = 3 * x0 - 6 * x1 + 3 * x2
    const cx = -3 * x0 + 3 * x1

    const ay = -y0 + 3 * y1 - 3 * y2 + y3
    const by = 3 * y0 - 6 * y1 + 3 * y2
    const cy = -3 * y0 + 3 * y1
    const dy = y0 - ymin

    // Ищем I = integral_0^1 (y(t)-ymin) * x'(t) dt
    // x'(t) = 3*ax*t^2 + 2*bx*t + cx
    // (ay t^3 + by t^2 + cy t + dy) * (3ax t^2 + 2bx t + cx)
    const m5 = ay * (3 * ax)
    const m4 = ay * (2 * bx) + by * (3 * ax)
    const m3 = ay * cx + by * (2 * bx) + cy * (3 * ax)
    const m2 = by * cx + cy * (2 * bx) + dy * (3 * ax)
    const m1 = cy * cx + dy * (2 * bx)
    const m0 = dy * cx

    return m5 / 6 + m4 / 5 + m3 / 4 + m2 / 3 + m1 / 2 + m0
  }

  /**
   * Return new segment transformed using affine transformation matrix
   */
  transform(matrix = new geom.Matrix()): Bezier {
    return new Bezier(
      this.start.transform(matrix),
      this.control1.transform(matrix),
      this.control2.transform(matrix),
      this.end.transform(matrix),
    )
  }

  /**
   * Returns true if segment start is equal to segment end up to DP_TOL
   */
  isZeroLength(): boolean {
    return this.start.equalTo(this.end) && this.start.equalTo(this.control1) && this.start.equalTo(this.control2)
  }

  get name() {
    return 'bezier'
  }
}

function pointAtRatio(start: Point, end: Point, f: number) {
  if (f <= 0) return start
  if (f >= 1.0) return end
  return new Point((end.x - start.x) * f + start.x, (end.y - start.y) * f + start.y)
}

function getSegmentIntersect(shape: Shape) {
  if (shape instanceof Segment) {
    return Intersection.intersectSegment2Segment
  }
  if (shape instanceof Arc) {
    return Intersection.intersectSegment2Arc
  }
  if (shape instanceof Bezier) {
    return Intersection.intersectSegment2Bezier
  }
  if (shape instanceof Quadratic) {
    return Intersection.intersectSegment2Quadratic
  }
  throw new Error('unimplemented')
}

function getSegmentDistance(shape: Shape): (s: Segment, o: any) => [number, Segment] {
  if (shape instanceof Point) {
    return Distance.segment2point
  }
  if (shape instanceof Segment) {
    return Distance.segment2segment
  }
  if (shape instanceof Arc) {
    return Distance.segment2arc
  }
  if (shape instanceof Quadratic) {
    return Distance.segment2quadratic
  }
  if (shape instanceof Bezier) {
    return Distance.segment2bezier
  }
  if (shape instanceof geom.Polygon) {
    return Distance.segment2polygon
  }
  throw new Error('unimplemented')
}

/**
 * Shortcut method to create new bezier
 */
export const bezier = (a: any, b: any, c: any, d: any) => new Bezier(a, b, c, d)

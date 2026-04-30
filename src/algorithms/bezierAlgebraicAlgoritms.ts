import * as Utils from '../utils/utils'
import * as g from '../classes'

/**
 * Algebraic methods for precise Bezier curve calculations
 */

/**
 * Solve cubic equation ax³ + bx² + cx + d = 0
 * Returns array of real roots in range [0, 1]
 */
export function solveCubic(a: number, b: number, c: number, d: number): number[] {
  const roots: number[] = []
  
  // Normalize coefficients
  if (Math.abs(a) < 1e-10) {
    // Quadratic equation
    return solveQuadratic(b, c, d)
  }
  
  // Convert to depressed cubic t³ + pt + q = 0
  const p = (3 * a * c - b * b) / (3 * a * a)
  const q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a)
  
  const discriminant = -(4 * p * p * p + 27 * q * q)
  
  if (discriminant > 0) {
    // Three real roots
    const m = 2 * Math.sqrt(-p / 3)
    const theta = Math.acos((3 * q) / (p * m)) / 3
    const offset = -b / (3 * a)
    
    for (let k = 0; k < 3; k++) {
      const root = m * Math.cos(theta - (2 * Math.PI * k) / 3) + offset
      if (root >= -1e-10 && root <= 1 + 1e-10) {
        roots.push(Math.max(0, Math.min(1, root)))
      }
    }
  } else if (Math.abs(discriminant) < 1e-10) {
    // One or two real roots
    const offset = -b / (3 * a)
    if (Math.abs(p) < 1e-10) {
      // One root
      if (offset >= -1e-10 && offset <= 1 + 1e-10) {
        roots.push(Math.max(0, Math.min(1, offset)))
      }
    } else {
      // Two roots
      const root1 = (3 * q) / p + offset
      const root2 = (-3 * q) / (2 * p) + offset
      
      if (root1 >= -1e-10 && root1 <= 1 + 1e-10) {
        roots.push(Math.max(0, Math.min(1, root1)))
      }
      if (root2 >= -1e-10 && root2 <= 1 + 1e-10) {
        roots.push(Math.max(0, Math.min(1, root2)))
      }
    }
  } else {
    // One real root (Cardano's formula)
    const sqrtD = Math.sqrt(-discriminant / 27)
    const u = Math.cbrt(-q / 2 + sqrtD)
    const v = Math.cbrt(-q / 2 - sqrtD)
    const root = u + v - b / (3 * a)
    
    if (root >= -1e-10 && root <= 1 + 1e-10) {
      roots.push(Math.max(0, Math.min(1, root)))
    }
  }
  
  return roots
}

/**
 * Solve quadratic equation ax² + bx + c = 0
 * Returns array of real roots in range [0, 1]
 */
export function solveQuadratic(a: number, b: number, c: number): number[] {
  const roots: number[] = []
  
  if (Math.abs(a) < 1e-10) {
    // Linear equation
    if (Math.abs(b) > 1e-10) {
      const root = -c / b
      if (root >= -1e-10 && root <= 1 + 1e-10) {
        roots.push(Math.max(0, Math.min(1, root)))
      }
    }
    return roots
  }
  
  const discriminant = b * b - 4 * a * c
  
  if (discriminant < 0) {
    return roots
  }
  
  if (Math.abs(discriminant) < 1e-10) {
    const root = -b / (2 * a)
    if (root >= -1e-10 && root <= 1 + 1e-10) {
      roots.push(Math.max(0, Math.min(1, root)))
    }
  } else {
    const sqrtD = Math.sqrt(discriminant)
    const root1 = (-b + sqrtD) / (2 * a)
    const root2 = (-b - sqrtD) / (2 * a)
    
    if (root1 >= -1e-10 && root1 <= 1 + 1e-10) {
      roots.push(Math.max(0, Math.min(1, root1)))
    }
    if (root2 >= -1e-10 && root2 <= 1 + 1e-10) {
      roots.push(Math.max(0, Math.min(1, root2)))
    }
  }
  
  return roots
}

/**
 * Evaluate cubic Bezier curve at parameter t
 */
export function bezierPoint(
  p0: g.Point,
  p1: g.Point,
  p2: g.Point,
  p3: g.Point,
  t: number
): g.Point {
  const t2 = t * t
  const t3 = t2 * t
  const mt = 1 - t
  const mt2 = mt * mt
  const mt3 = mt2 * mt
  
  const x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x
  const y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y
  
  return new g.Point(x, y)
}

/**
 * Evaluate first derivative of cubic Bezier curve at parameter t
 */
export function bezierDerivative(
  p0: g.Point,
  p1: g.Point,
  p2: g.Point,
  p3: g.Point,
  t: number
): g.Vector {
  const mt = 1 - t
  const mt2 = mt * mt
  const t2 = t * t
  
  // B'(t) = 3(1-t)²(P1-P0) + 6(1-t)t(P2-P1) + 3t²(P3-P2)
  const dx =
    3 * mt2 * (p1.x - p0.x) +
    6 * mt * t * (p2.x - p1.x) +
    3 * t2 * (p3.x - p2.x)
  
  const dy =
    3 * mt2 * (p1.y - p0.y) +
    6 * mt * t * (p2.y - p1.y) +
    3 * t2 * (p3.y - p2.y)
  
  return new g.Vector(dx, dy)
}

/**
 * Find closest point on Bezier curve to a given point using algebraic method
 * Returns [distance, segment, parameter t]
 */
export function bezierClosestPoint(
  bezier: g.Bezier,
  point: g.Point
): [number, g.Segment, number] {
  const p0 = bezier.start
  const p1 = bezier.control1
  const p2 = bezier.control2
  const p3 = bezier.end
  
  // We need to minimize distance² = (B(t) - P)²
  // Taking derivative and setting to 0: (B(t) - P) · B'(t) = 0
  // This gives us a 5th degree polynomial, but we can solve it numerically
  
  // First, check endpoints
  let minDist = point.distanceTo(p0)[0]
  let minT = 0
  let minPoint = p0
  
  const distEnd = point.distanceTo(p3)[0]
  if (distEnd < minDist) {
    minDist = distEnd
    minT = 1
    minPoint = p3
  }
  
  // Find critical points using Newton-Raphson method
  // Start with multiple initial guesses
  const initialGuesses = [0.25, 0.5, 0.75]
  
  for (const t0 of initialGuesses) {
    let t = t0
    
    // Newton-Raphson iterations
    for (let iter = 0; iter < 20; iter++) {
      const B = bezierPoint(p0, p1, p2, p3, t)
      const Bp = bezierDerivative(p0, p1, p2, p3, t)
      
      // f(t) = (B(t) - P) · B'(t)
      const diff = new g.Vector(point, B)
      const f = diff.dot(Bp)
      
      if (Math.abs(f) < 1e-10) break
      
      // f'(t) = B'(t) · B'(t) + (B(t) - P) · B''(t)
      // For simplicity, approximate B''(t)
      const eps = 1e-6
      const tPlus = Math.min(1, t + eps)
      const tMinus = Math.max(0, t - eps)
      const Bp_plus = bezierDerivative(p0, p1, p2, p3, tPlus)
      const Bp_minus = bezierDerivative(p0, p1, p2, p3, tMinus)
      const Bpp_approx = new g.Vector(
        (Bp_plus.x - Bp_minus.x) / (tPlus - tMinus),
        (Bp_plus.y - Bp_minus.y) / (tPlus - tMinus)
      )
      
      const fp = Bp.dot(Bp) + diff.dot(Bpp_approx)
      
      if (Math.abs(fp) < 1e-10) {
        t = Math.max(0, Math.min(1, t - 0.1 * Math.sign(f)))
        continue
      }
      
      const tNew = t - f / fp
      
      if (Math.abs(tNew - t) < 1e-10) break
      
      t = Math.max(0, Math.min(1, tNew))
    }
    
    // Check if this is a better solution
    if (t >= 0 && t <= 1) {
      const candidatePoint = bezierPoint(p0, p1, p2, p3, t)
      const dist = point.distanceTo(candidatePoint)[0]
      
      if (dist < minDist) {
        minDist = dist
        minT = t
        minPoint = candidatePoint
      }
    }
  }
  
  return [minDist, new g.Segment(minPoint, point), minT]
}

/**
 * Check if a point lies on the Bezier curve using algebraic method
 * Solves the cubic equation B(t) = point for t ∈ [0, 1]
 */
export function bezierContainsPoint(
  bezier: g.Bezier,
  point: g.Point,
  tolerance: number = 1e-6
): boolean {
  const p0 = bezier.start
  const p1 = bezier.control1
  const p2 = bezier.control2
  const p3 = bezier.end
  
  // Cubic Bezier: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
  // Expanding: B(t) = at³ + bt² + ct + d
  // where:
  //   a = P₃ - 3P₂ + 3P₁ - P₀
  //   b = 3P₂ - 6P₁ + 3P₀
  //   c = 3P₁ - 3P₀
  //   d = P₀
  
  // Coefficients for x coordinate
  const ax = p3.x - 3 * p2.x + 3 * p1.x - p0.x
  const bx = 3 * p2.x - 6 * p1.x + 3 * p0.x
  const cx = 3 * p1.x - 3 * p0.x
  const dx = p0.x - point.x
  
  // Coefficients for y coordinate
  const ay = p3.y - 3 * p2.y + 3 * p1.y - p0.y
  const by = 3 * p2.y - 6 * p1.y + 3 * p0.y
  const cy = 3 * p1.y - 3 * p0.y
  const dy = p0.y - point.y
  
  // Solve cubic equation for x: ax*t³ + bx*t² + cx*t + dx = 0
  const rootsX = solveCubic(ax, bx, cx, dx)
  
  // For each root from x equation, check if it also satisfies y equation
  for (const t of rootsX) {
    // Evaluate y at this t value
    const yValue = ay * t * t * t + by * t * t + cy * t + dy
    
    // If y equation is also satisfied (within tolerance), point is on curve
    if (Math.abs(yValue) < tolerance) {
      return true
    }
  }
  
  // Alternative: solve for y and check x (in case x equation is degenerate)
  const rootsY = solveCubic(ay, by, cy, dy)
  
  for (const t of rootsY) {
    const xValue = ax * t * t * t + bx * t * t + cx * t + dx
    
    if (Math.abs(xValue) < tolerance) {
      return true
    }
  }
  
  return false
}
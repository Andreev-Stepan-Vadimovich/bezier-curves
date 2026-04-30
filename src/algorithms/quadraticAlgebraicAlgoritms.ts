import * as g from '../classes'
import { solveCubic, solveQuadratic } from './solveEquations'

/**
 * Algebraic methods for precise Quadratic Bezier curve calculations
 * Quadratic Bezier: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂, t ∈ [0, 1]
 */

/**
 * Evaluate quadratic Bezier curve at parameter t
 * B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
 */
export function quadraticPoint(
  p0: g.Point,
  p1: g.Point,
  p2: g.Point,
  t: number
): g.Point {
  const mt = 1 - t
  const mt2 = mt * mt
  const t2 = t * t
  
  const x = mt2 * p0.x + 2 * mt * t * p1.x + t2 * p2.x
  const y = mt2 * p0.y + 2 * mt * t * p1.y + t2 * p2.y
  
  return new g.Point(x, y)
}

/**
 * Evaluate first derivative of quadratic Bezier curve at parameter t
 * B'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁) = 2[(P₁-P₀) + t(P₂-2P₁+P₀)]
 */
export function quadraticDerivative(
  p0: g.Point,
  p1: g.Point,
  p2: g.Point,
  t: number
): g.Vector {
  const mt = 1 - t
  
  // B'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁)
  const dx = 2 * (mt * (p1.x - p0.x) + t * (p2.x - p1.x))
  const dy = 2 * (mt * (p1.y - p0.y) + t * (p2.y - p1.y))
  
  return new g.Vector(dx, dy)
}

/**
 * Evaluate second derivative of quadratic Bezier curve (constant)
 * B''(t) = 2(P₂ - 2P₁ + P₀)
 */
export function quadraticSecondDerivative(
  p0: g.Point,
  p1: g.Point,
  p2: g.Point
): g.Vector {
  const dx = 2 * (p2.x - 2 * p1.x + p0.x)
  const dy = 2 * (p2.y - 2 * p1.y + p0.y)
  
  return new g.Vector(dx, dy)
}

/**
 * Find closest point on quadratic Bezier curve to a given point using algebraic method
 * 
 * Mathematical approach:
 * Minimize D²(t) = ||B(t) - P||²
 * Critical points: d/dt[D²(t)] = 2(B(t) - P) · B'(t) = 0
 * 
 * For quadratic Bezier, this yields a CUBIC equation in t, which we solve algebraically.
 * 
 * Returns [distance, segment, parameter t]
 */
export function quadraticClosestPoint(
  bezier: g.Quadratic,
  point: g.Point
): [number, g.Segment, number] {
  const p0 = bezier.start
  const p1 = bezier.control1
  const p2 = bezier.end
  
  // First, check endpoints
  let minDist = point.distanceTo(p0)[0]
  let minT = 0
  let minPoint = p0
  
  const distEnd = point.distanceTo(p2)[0]
  if (distEnd < minDist) {
    minDist = distEnd
    minT = 1
    minPoint = p2
  }
  
  // ============================================================================
  // Algebraic solution: solve (B(t) - P) · B'(t) = 0
  // This expands to a cubic equation: At³ + Bt² + Ct + D = 0
  // ============================================================================
  
  // Precompute differences for cleaner coefficient calculation
  const A = new g.Vector(p0.x - 2 * p1.x + p2.x, p0.y - 2 * p1.y + p2.y) // P₀ - 2P₁ + P₂
  const B = new g.Vector(2 * p1.x - 2 * p0.x, 2 * p1.y - 2 * p0.y)         // 2(P₁ - P₀)
  const C = new g.Vector(p0.x - point.x, p0.y - point.y)                    // P₀ - P
  
  // B(t) = A·t² + B·t + C (in vector form, with redefined coefficients)
  // Actually, let's use the standard form:
  // B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
  //      = P₀ + 2t(P₁-P₀) + t²(P₂-2P₁+P₀)
  // Let: Q₀ = P₀, Q₁ = 2(P₁-P₀), Q₂ = P₂-2P₁+P₀
  // Then: B(t) = Q₀ + Q₁·t + Q₂·t²
  
  const Q0 = p0
  const Q1 = new g.Vector(2 * (p1.x - p0.x), 2 * (p1.y - p0.y))
  const Q2 = new g.Vector(p2.x - 2 * p1.x + p0.x, p2.y - 2 * p1.y + p0.y)
  
  // B'(t) = Q₁ + 2·Q₂·t
  
  // (B(t) - P) · B'(t) = 0
  // ((Q₀-P) + Q₁·t + Q₂·t²) · (Q₁ + 2·Q₂·t) = 0
  
  const Q0P = new g.Vector(Q0.x - point.x, Q0.y - point.y)
  
  // Expand the dot product:
  // (Q0P + Q1·t + Q2·t²) · (Q1 + 2·Q2·t) = 0
  // = Q0P·Q1 + 2·Q0P·Q2·t + Q1·Q1·t + 2·Q1·Q2·t² + Q2·Q1·t² + 2·Q2·Q2·t³ = 0
  // = 2(Q2·Q2)·t³ + (3·Q1·Q2)·t² + (2·Q0P·Q2 + Q1·Q1)·t + (Q0P·Q1) = 0
  
  const a = 2 * Q2.dot(Q2)
  const b = 3 * Q1.dot(Q2)
  const c = 2 * Q0P.dot(Q2) + Q1.dot(Q1)
  const d = Q0P.dot(Q1)
  
  // Solve the cubic equation
  const roots = solveCubic(a, b, c, d)
  
  // Evaluate distance at each critical point
  for (const t of roots) {
    if (t < 0 || t > 1) continue
    
    const candidatePoint = quadraticPoint(p0, p1, p2, t)
    const dist = point.distanceTo(candidatePoint)[0]
    
    if (dist < minDist) {
      minDist = dist
      minT = t
      minPoint = candidatePoint
    }
  }
  
  // Fallback: Newton-Raphson refinement if algebraic solution is unstable
  // (e.g., when coefficients are near-zero due to degenerate curve)
  if (Math.abs(a) < 1e-12 && Math.abs(b) < 1e-12) {
    // Degenerate to linear or constant - use numerical method
    const initialGuesses = [0.25, 0.5, 0.75]
    
    for (const t0 of initialGuesses) {
      let t = t0
      
      for (let iter = 0; iter < 15; iter++) {
        const B = quadraticPoint(p0, p1, p2, t)
        const Bp = quadraticDerivative(p0, p1, p2, t)
        const diff = new g.Vector(point.x - B.x, point.y - B.y)
        
        const f = -diff.dot(Bp) // We want (B-P)·B' = 0, so f = -(B-P)·B'
        
        if (Math.abs(f) < 1e-12) break
        
        // f'(t) = -[B'·B' + (B-P)·B'']
        const Bpp = quadraticSecondDerivative(p0, p1, p2)
        const fp = -(Bp.dot(Bp) + diff.dot(Bpp))
        
        if (Math.abs(fp) < 1e-12) {
          t = Math.max(0, Math.min(1, t - 0.05 * Math.sign(f)))
          continue
        }
        
        const tNew = t - f / fp
        if (Math.abs(tNew - t) < 1e-12) break
        
        t = Math.max(0, Math.min(1, tNew))
      }
      
      const candidatePoint = quadraticPoint(p0, p1, p2, t)
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
 * Check if a point lies on the quadratic Bezier curve using algebraic method
 * Solves B(t) = point for t ∈ [0, 1]
 */
export function quadraticContainsPoint(
  bezier: g.Quadratic,
  point: g.Point,
  tolerance: number = 1e-6
): boolean {
  const p0 = bezier.start
  const p1 = bezier.control1
  const p2 = bezier.end
  
  // Quadratic Bezier: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
  // Expanding: B(t) = at² + bt + c where:
  //   a = P₂ - 2P₁ + P₀
  //   b = 2(P₁ - P₀)
  //   c = P₀
  
  const ax = p2.x - 2 * p1.x + p0.x
  const bx = 2 * (p1.x - p0.x)
  const cx = p0.x - point.x
  
  const ay = p2.y - 2 * p1.y + p0.y
  const by = 2 * (p1.y - p0.y)
  const cy = p0.y - point.y
  
  // Solve quadratic equation for x: ax·t² + bx·t + cx = 0
  const rootsX = solveQuadratic(ax, bx, cx)
  
  // Check if any root also satisfies y equation
  for (const t of rootsX) {
    const yValue = ay * t * t + by * t + cy
    if (Math.abs(yValue) < tolerance) {
      return true
    }
  }
  
  // Fallback: solve for y and check x (handles degenerate x-equation)
  const rootsY = solveQuadratic(ay, by, cy)
  
  for (const t of rootsY) {
    const xValue = ax * t * t + bx * t + cx
    if (Math.abs(xValue) < tolerance) {
      return true
    }
  }
  
  return false
}

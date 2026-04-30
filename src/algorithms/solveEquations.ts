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
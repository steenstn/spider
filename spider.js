// https://www.researchgate.net/publication/220632147_FABRIK_A_fast_iterative_solver_for_the_Inverse_Kinematics_problem
// https://www.youtube.com/watch?v=UNoX65PRehA&t=944s
// https://github.com/lo-th/fullik/blob/gh-pages/src/core/Joint2D.js

let armLength = 50;

let distance = (a, b) => {
    return Math.sqrt((a.x - b.x)*(a.x-b.x) + (a.y - b.y)*(a.y-b.y));
};

let getDistancesBetweenPoints = (points) => {
  let distances = []
  for(let i = 0; i < points.length-1; i++) {
    distances.push(distance(points[i], points[i+1]));
  }
  return distances;
}

let easeOutQuad = (progress) => {
    return 1 - (1 - progress) * (1 - progress);
}

let lerp = (startValue, endValue, progress) => {
    return startValue + (endValue - startValue) * progress
}

let fabrik = (positions, distancesBetweenJoints, target) => {
  let distanceToTarget = distance(positions[0], target);
  let jointDistanceSum = distancesBetweenJoints.reduce( (a,b) => {return a+b}, 0);
  if (distanceToTarget > jointDistanceSum) { // Target unreachable
    for(let i = 0; i < positions.length-1; i++) {
      let r = distance(target, positions[i]);
      let delta = distancesBetweenJoints[i]/r;
      positions[i+1].x = (1-delta)*positions[i].x+delta*target.x;
      positions[i+1].y = (1-delta)*positions[i].y+delta*target.y;
    }
  } else {
    let bx = positions[0].x;
    let by = positions[0].y;
    let diff = distance(positions[positions.length-1], target);
    for(let iter = 0; iter < 3; iter++) {
      // Stage 1: Forward reaching
      positions[positions.length-1].x = target.x;
      positions[positions.length-1].y = target.y;

      for(let i = positions.length-2; i >=0; i--) {
        let r = distance(positions[i+1], positions[i]);
        let delta = distancesBetweenJoints[i]/r;
        positions[i].x = (1-delta)*positions[i+1].x+delta*positions[i].x;
        positions[i].y = (1-delta)*positions[i+1].y+delta*positions[i].y;
      }

      // Stage 2: Backward reaching
      positions[0].x = bx;
      positions[0].y = by;
    
      for(let i = 0; i < positions.length-2; i++) {
        let r = distance(positions[i+1], positions[i]);
        let delta = distancesBetweenJoints[i]/r;
        positions[i+1].x = (1-delta)*positions[i].x + delta*positions[i+1].x;
        positions[i+1].y = (1-delta)*positions[i].y + delta*positions[i+1].y;
      }
      diff = distance(positions[positions.length-1], target);
    }
  }

}

/*
 This method drags to the target. No fix point
 */
let fabrik2 = (positions, distancesBetweenJoints, target) => {
  let distanceToTarget = distance(positions[0], target);
  let jointDistanceSum = distancesBetweenJoints.reduce( (a,b) => {return a+b}, 0);
  let bx = positions[0].x;
  let by = positions[0].y;
  let diff = distance(positions[positions.length-1], target);

  positions[positions.length-1].x = target.x;
  positions[positions.length-1].y = target.y;

  for(let i = positions.length-2; i >=0; i--) {
    let r = distance(positions[i+1], positions[i]);
    let delta = distancesBetweenJoints[i]/r;
    positions[i].x = (1-delta)*positions[i+1].x+delta*positions[i].x;
    positions[i].y = (1-delta)*positions[i+1].y+delta*positions[i].y;
  }
}

class Spider {
  constructor(startX, startY) {
    this.arms = [];
    this.distances = [];
    this.targets = [];
    let numArms = 3;
    let arm_length = 30;
    for(let i = 0; i < numArms; i++) {
      let arm = [{x: startX, y:startY}];
      for(let j =1; j<=3;j++) {
        arm.push({x:startX+40+j*arm_length,y:startY+j*arm_length});
      }
      this.arms.push(arm);
      this.distances.push(getDistancesBetweenPoints(arm))
      this.targets.push({x:this.arms[i][numArms-1].x+i*40, y:this.arms[i][numArms-1].y+40});
    }
    this.body = this.arms[0][0];
  }

  moveBody(x, y) {
    this.arms.forEach(arm => {
      arm[0].x = x;
      arm[0].y = y;
    })
  }
}

let updateTargets = (targets, origin) => {
  targets.forEach((t, i) => {
    let normalizedI = i-(targets.length/2)+0.5;
    //
    //console.log(i, normalizedI)
    if(Math.abs(t.x+20*normalizedI -origin.x) > 90) {
      let diff = t.x - origin.x;
      t.x-=diff*1.5;
    }
  });
}

let getNewTargets = (targets, origin) => {
  // TODO allocate right away
  let newTargets = [];

  targets.forEach((t, i) => {
      let normalizedI = i/(targets.length);
      let optimalDiff = origin.x-40+normalizedI*150;
      let diff = origin.x-t.x;
        newTargets.push({x:optimalDiff, y:400});
  });
  return newTargets;
}

/*
 Interpolate between p0, p1, p2 with with t = 0-1
 */
let bezier = (t, p0, p1, p2) => {
  return (1-t)*((1-t)*p0 + t*p1) + t*((1-t)*p1 +t*p2);
}



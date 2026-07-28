/**
 * @module render/DebugDraw
 *
 * Canvas 2D debug renderer.
 *
 * Draws shapes, contacts, joints, AABBs, centres of mass and the broad-phase
 * tree straight from the world. Entirely optional — nothing in the simulation
 * depends on it, and tree-shaking removes it from a production bundle if you
 * never import it.
 *
 * ```ts
 * const draw = new DebugDraw(canvas.getContext('2d')!, { pixelsPerMeter: 32 });
 * draw.flags.shapes = true;
 * draw.flags.contacts = true;
 *
 * function frame() {
 *   world.step();
 *   draw.begin();
 *   draw.drawWorld(world);
 *   draw.end();
 *   requestAnimationFrame(frame);
 * }
 * ```
 */

import * as S from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { Transform } from './../math/Transform.js';
import { AABB } from './../collision/AABB.js';
import { ShapeType } from './../collision/Shape.js';
import type { Shape } from './../collision/Shape.js';
import type { Circle } from './../collision/shapes/Circle.js';
import type { Capsule } from './../collision/shapes/Capsule.js';
import type { Polygon } from './../collision/shapes/Polygon.js';
import type { Segment } from './../collision/shapes/Segment.js';
import type { World } from './../dynamics/World.js';
import { BodyType } from './../dynamics/Body.js';
import type { Body } from './../dynamics/Body.js';

/** Which layers to draw. */
export interface DebugDrawFlags {
  shapes: boolean;
  /** Fill shapes as well as outlining them. */
  fill: boolean;
  joints: boolean;
  contacts: boolean;
  contactNormals: boolean;
  contactImpulses: boolean;
  aabbs: boolean;
  centerOfMass: boolean;
  /** Grey out sleeping bodies. */
  sleepState: boolean;
  /** Velocity arrows. */
  velocities: boolean;
  /** A small stats overlay. */
  stats: boolean;
}

/** Colour palette; every entry is a CSS colour string. */
export interface DebugDrawColors {
  staticBody: string;
  kinematicBody: string;
  dynamicBody: string;
  sleepingBody: string;
  sensor: string;
  joint: string;
  contactPoint: string;
  contactNormal: string;
  contactImpulse: string;
  aabb: string;
  centerOfMass: string;
  velocity: string;
  text: string;
}

/** Renderer options. */
export interface DebugDrawOptions {
  /** Zoom: how many pixels one metre occupies. Default `32`. */
  pixelsPerMeter?: number;
  /** World point shown at the canvas centre. */
  cameraX?: number;
  cameraY?: number;
  /** Line width in **pixels**, kept constant regardless of zoom. Default `1.5`. */
  lineWidth?: number;
  flags?: Partial<DebugDrawFlags>;
  colors?: Partial<DebugDrawColors>;
}

const DEFAULT_COLORS: DebugDrawColors = {
  staticBody: '#7ec850',
  kinematicBody: '#5ba8d8',
  dynamicBody: '#e8a33d',
  sleepingBody: '#7d7d8c',
  sensor: '#c45bd8',
  joint: '#d8d85b',
  contactPoint: '#ff4d4d',
  contactNormal: '#ffffff',
  contactImpulse: '#ff9d3d',
  aabb: 'rgba(120,200,255,0.35)',
  centerOfMass: '#ff2d95',
  velocity: '#4dffc3',
  text: '#e8e8ef',
};

const _p = Vec2.zero();
const _p2 = Vec2.zero();
const _aabb = new AABB();
const _v = Vec2.zero();

export class DebugDraw {
  readonly ctx: CanvasRenderingContext2D;
  /** Zoom, pixels per metre. */
  pixelsPerMeter: number;
  /** Camera centre, world space. */
  cameraX: number;
  cameraY: number;
  lineWidth: number;

  readonly flags: DebugDrawFlags = {
    shapes: true,
    fill: true,
    joints: true,
    contacts: false,
    contactNormals: false,
    contactImpulses: false,
    aabbs: false,
    centerOfMass: false,
    sleepState: true,
    velocities: false,
    stats: false,
  };

  readonly colors: DebugDrawColors;

  constructor(ctx: CanvasRenderingContext2D, options: DebugDrawOptions = {}) {
    this.ctx = ctx;
    this.pixelsPerMeter = options.pixelsPerMeter ?? 32;
    this.cameraX = options.cameraX ?? 0;
    this.cameraY = options.cameraY ?? 0;
    this.lineWidth = options.lineWidth ?? 1.5;
    if (options.flags) Object.assign(this.flags, options.flags);
    this.colors = { ...DEFAULT_COLORS, ...options.colors };
  }

  /* -------------------------- transform -------------------------- */

  /**
   * Install the world→screen transform.
   *
   * The y axis is flipped so that **+y is up**, matching the physics
   * convention rather than the canvas one.
   */
  begin(clear = true): void {
    const ctx = this.ctx;
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.save();
    if (clear) ctx.clearRect(0, 0, w, h);
    ctx.translate(w * 0.5, h * 0.5);
    ctx.scale(this.pixelsPerMeter, -this.pixelsPerMeter);
    ctx.translate(-this.cameraX, -this.cameraY);
    ctx.lineWidth = this.lineWidth / this.pixelsPerMeter;
    ctx.lineJoin = 'round';
  }

  /** Restore the canvas state. */
  end(): void {
    this.ctx.restore();
  }

  /** Convert a canvas pixel coordinate to a world coordinate. */
  screenToWorld(px: number, py: number): { x: number; y: number } {
    const c = this.ctx.canvas;
    return {
      x: (px - c.width * 0.5) / this.pixelsPerMeter + this.cameraX,
      y: -(py - c.height * 0.5) / this.pixelsPerMeter + this.cameraY,
    };
  }

  /** Convert a world coordinate to a canvas pixel coordinate. */
  worldToScreen(x: number, y: number): { x: number; y: number } {
    const c = this.ctx.canvas;
    return {
      x: (x - this.cameraX) * this.pixelsPerMeter + c.width * 0.5,
      y: -(y - this.cameraY) * this.pixelsPerMeter + c.height * 0.5,
    };
  }

  /* --------------------------- world ----------------------------- */

  /** Draw every enabled layer of a world. */
  drawWorld(world: World): void {
    if (this.flags.shapes) {
      for (const body of world.eachBody()) this.drawBody(body);
    }
    if (this.flags.aabbs) {
      for (const body of world.eachBody()) {
        for (const f of body.fixtures) {
          if (f.proxyId < 0) continue;
          world.broadPhase.tree.getAABB(_aabb, f.proxyId);
          this.strokeAABB(_aabb, this.colors.aabb);
        }
      }
    }
    if (this.flags.joints) {
      for (const joint of world.eachJoint()) this.drawJoint(joint);
    }
    if (this.flags.contacts || this.flags.contactNormals || this.flags.contactImpulses) {
      this.drawContacts(world);
    }
    if (this.flags.centerOfMass) {
      for (const body of world.eachBody()) {
        if (body.type !== BodyType.Dynamic) continue;
        this.drawCross(body.worldCenter, 0.15, this.colors.centerOfMass);
      }
    }
    if (this.flags.velocities) {
      for (const body of world.eachBody()) {
        if (body.type === BodyType.Static || !body.awake) continue;
        _v.set(
          body.worldCenter.x + S.mul(body.linearVelocity.x, S.fromFloat(0.15)),
          body.worldCenter.y + S.mul(body.linearVelocity.y, S.fromFloat(0.15)),
        );
        this.drawArrow(body.worldCenter, _v, this.colors.velocity);
      }
    }
    if (this.flags.stats) this.drawStats(world);
  }

  /** Draw one body's fixtures. */
  drawBody(body: Body): void {
    const color = this.bodyColor(body);
    for (const f of body.fixtures) {
      this.drawShape(f.shape, body.transform, f.isSensor ? this.colors.sensor : color, f.isSensor);
    }
  }

  private bodyColor(body: Body): string {
    if (this.flags.sleepState && !body.awake && body.type !== BodyType.Static) {
      return this.colors.sleepingBody;
    }
    switch (body.type) {
      case BodyType.Static:
        return this.colors.staticBody;
      case BodyType.Kinematic:
        return this.colors.kinematicBody;
      default:
        return this.colors.dynamicBody;
    }
  }

  /* --------------------------- shapes ---------------------------- */

  /** Draw a single shape under a transform. */
  drawShape(shape: Shape, xf: Transform, color: string, dashed = false): void {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = this.flags.fill ? withAlpha(color, 0.28) : 'transparent';
    if (dashed) ctx.setLineDash([0.12, 0.1]);

    switch (shape.type) {
      case ShapeType.Circle:
        this.pathCircle(shape as Circle, xf);
        break;
      case ShapeType.Capsule:
        this.pathCapsule(shape as Capsule, xf);
        break;
      case ShapeType.Polygon:
        this.pathPolygon(shape as Polygon, xf);
        break;
      default:
        this.pathSegment(shape as Segment, xf);
        break;
    }

    if (this.flags.fill && shape.type !== ShapeType.Segment) ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  private pathCircle(c: Circle, xf: Transform): void {
    const ctx = this.ctx;
    Transform.apply(_p, xf, c.center);
    const x = S.toFloat(_p.x);
    const y = S.toFloat(_p.y);
    const r = S.toFloat(c.radius);
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.closePath();
    // A radius line makes rotation visible on an otherwise featureless disc.
    ctx.moveTo(x, y);
    ctx.lineTo(x + r * S.toFloat(xf.q.c), y + r * S.toFloat(xf.q.s));
  }

  private pathCapsule(c: Capsule, xf: Transform): void {
    const ctx = this.ctx;
    Transform.apply(_p, xf, c.p1);
    Transform.apply(_p2, xf, c.p2);
    const x1 = S.toFloat(_p.x);
    const y1 = S.toFloat(_p.y);
    const x2 = S.toFloat(_p2.x);
    const y2 = S.toFloat(_p2.y);
    const r = S.toFloat(c.radius);
    const angle = Math.atan2(y2 - y1, x2 - x1);
    ctx.beginPath();
    ctx.arc(x1, y1, r, angle + Math.PI / 2, angle - Math.PI / 2);
    ctx.arc(x2, y2, r, angle - Math.PI / 2, angle + Math.PI / 2);
    ctx.closePath();
  }

  private pathPolygon(p: Polygon, xf: Transform): void {
    const ctx = this.ctx;
    ctx.beginPath();
    for (let i = 0; i < p.vertexCount; i++) {
      Transform.apply(_p, xf, p.vertices[i]!);
      const x = S.toFloat(_p.x);
      const y = S.toFloat(_p.y);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.closePath();
  }

  private pathSegment(s: Segment, xf: Transform): void {
    const ctx = this.ctx;
    Transform.apply(_p, xf, s.p1);
    Transform.apply(_p2, xf, s.p2);
    ctx.beginPath();
    ctx.moveTo(S.toFloat(_p.x), S.toFloat(_p.y));
    ctx.lineTo(S.toFloat(_p2.x), S.toFloat(_p2.y));
  }

  /* --------------------------- overlays -------------------------- */

  /** Draw contact points, normals and impulse magnitudes. */
  drawContacts(world: World): void {
    const ctx = this.ctx;
    ctx.save();
    for (const c of world.contacts) {
      if (!c.isTouching || c.isSensor) continue;
      const m = c.manifold;
      for (let i = 0; i < m.pointCount; i++) {
        const mp = m.points[i]!;
        if (this.flags.contacts) {
          this.fillDot(mp.point, 0.06, this.colors.contactPoint);
        }
        if (this.flags.contactNormals) {
          _p.set(
            mp.point.x + S.mul(m.normal.x, S.fromFloat(0.35)),
            mp.point.y + S.mul(m.normal.y, S.fromFloat(0.35)),
          );
          this.drawArrow(mp.point, _p, this.colors.contactNormal);
        }
        if (this.flags.contactImpulses) {
          const scale = S.toFloat(mp.normalImpulse) * 0.1;
          _p.set(
            mp.point.x + S.mul(m.normal.x, S.fromFloat(scale)),
            mp.point.y + S.mul(m.normal.y, S.fromFloat(scale)),
          );
          this.drawArrow(mp.point, _p, this.colors.contactImpulse);
        }
      }
    }
    ctx.restore();
  }

  /** Draw a joint as a line between its two anchors. */
  drawJoint(joint: { getAnchorA(o: Vec2): Vec2; getAnchorB(o: Vec2): Vec2 }): void {
    joint.getAnchorA(_p);
    joint.getAnchorB(_p2);
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = this.colors.joint;
    ctx.beginPath();
    ctx.moveTo(S.toFloat(_p.x), S.toFloat(_p.y));
    ctx.lineTo(S.toFloat(_p2.x), S.toFloat(_p2.y));
    ctx.stroke();
    ctx.restore();
    this.fillDot(_p, 0.05, this.colors.joint);
    this.fillDot(_p2, 0.05, this.colors.joint);
  }

  /** Draw the broad-phase tree's internal nodes — useful for tuning. */
  drawTree(world: World): void {
    const tree = world.broadPhase.tree;
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = this.colors.aabb;
    for (let i = 0; i < world.fixtures.length; i++) {
      const f = world.fixtures[i];
      if (!f || f.proxyId < 0) continue;
      tree.getAABB(_aabb, f.proxyId);
      this.strokeAABB(_aabb, this.colors.aabb);
    }
    ctx.restore();
  }

  /* --------------------------- helpers --------------------------- */

  /** Outline an AABB. */
  strokeAABB(aabb: AABB, color: string): void {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = color;
    const x = S.toFloat(aabb.lower.x);
    const y = S.toFloat(aabb.lower.y);
    ctx.strokeRect(
      x,
      y,
      S.toFloat(aabb.upper.x) - x,
      S.toFloat(aabb.upper.y) - y,
    );
    ctx.restore();
  }

  /** A filled circle in world units. */
  fillDot(p: Vec2, radius: number, color: string): void {
    const ctx = this.ctx;
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(S.toFloat(p.x), S.toFloat(p.y), radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  /** An X marker. */
  drawCross(p: Vec2, size: number, color: string): void {
    const ctx = this.ctx;
    const x = S.toFloat(p.x);
    const y = S.toFloat(p.y);
    ctx.save();
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(x - size, y);
    ctx.lineTo(x + size, y);
    ctx.moveTo(x, y - size);
    ctx.lineTo(x, y + size);
    ctx.stroke();
    ctx.restore();
  }

  /** A line with an arrowhead at `to`. */
  drawArrow(from: Vec2, to: Vec2, color: string): void {
    const ctx = this.ctx;
    const x1 = S.toFloat(from.x);
    const y1 = S.toFloat(from.y);
    const x2 = S.toFloat(to.x);
    const y2 = S.toFloat(to.y);
    const dx = x2 - x1;
    const dy = y2 - y1;
    const len = Math.sqrt(dx * dx + dy * dy);
    if (len < 1e-6) return;
    const ux = dx / len;
    const uy = dy / len;
    const head = Math.min(0.12, len * 0.4);

    ctx.save();
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2 - head * (ux + uy * 0.5), y2 - head * (uy - ux * 0.5));
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2 - head * (ux - uy * 0.5), y2 - head * (uy + ux * 0.5));
    ctx.stroke();
    ctx.restore();
  }

  /**
   * A small stats overlay, drawn in **screen** space so it is not affected by
   * the camera transform.
   */
  drawStats(world: World): void {
    const ctx = this.ctx;
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = this.colors.text;
    ctx.font = '12px ui-monospace, monospace';
    ctx.textBaseline = 'top';
    const lines = [
      `tick    ${world.tick}`,
      `bodies  ${world.bodyCount} (${world.awakeBodyCount} awake)`,
      `contacts${world.contactCount} (${world.profile.activeContactCount} active)`,
      `joints  ${world.jointCount}`,
      `step    ${world.profile.total.toFixed(2)} ms`,
      `  broad ${world.profile.broadPhase.toFixed(2)} ms`,
      `  narrow${world.profile.narrowPhase.toFixed(2)} ms`,
      `  solve ${world.profile.solve.toFixed(2)} ms`,
    ];
    let y = 8;
    for (const line of lines) {
      ctx.fillText(line, 8, y);
      y += 14;
    }
    ctx.restore();
  }
}

/** Add an alpha channel to a hex or `rgb()` colour. */
function withAlpha(color: string, alpha: number): string {
  if (color.startsWith('#')) {
    const hex = color.slice(1);
    const full =
      hex.length === 3
        ? hex
            .split('')
            .map((c) => c + c)
            .join('')
        : hex;
    const r = parseInt(full.slice(0, 2), 16);
    const g = parseInt(full.slice(2, 4), 16);
    const b = parseInt(full.slice(4, 6), 16);
    return `rgba(${r},${g},${b},${alpha})`;
  }
  if (color.startsWith('rgb(')) return color.replace('rgb(', 'rgba(').replace(')', `,${alpha})`);
  return color;
}


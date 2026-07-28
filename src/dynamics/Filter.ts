/**
 * @module dynamics/Filter
 *
 * Collision filtering — deciding which fixtures are allowed to touch.
 *
 * Two independent mechanisms, evaluated in this order:
 *
 * 1. **Groups.** Two fixtures with the same non-zero `group` always collide
 *    (positive) or never collide (negative), overriding the masks. Handy for
 *    "all parts of this ragdoll ignore each other".
 * 2. **Category / mask bitfields.** Otherwise both directions must agree:
 *    `A.mask & B.category` and `B.mask & A.category` must both be non-zero.
 */

/** Immutable filter description. */
export interface Filter {
  /** Which categories this fixture belongs to (bitfield). */
  category: number;
  /** Which categories this fixture collides with (bitfield). */
  mask: number;
  /** Override group; see the module docs. */
  group: number;
}

/** Collides with everything. */
export const DEFAULT_FILTER: Readonly<Filter> = Object.freeze({
  category: 0x0001,
  mask: 0xffff,
  group: 0,
});

/** Build a filter, filling in the defaults. */
export function makeFilter(partial?: Partial<Filter>): Filter {
  return {
    category: partial?.category ?? DEFAULT_FILTER.category,
    mask: partial?.mask ?? DEFAULT_FILTER.mask,
    group: partial?.group ?? DEFAULT_FILTER.group,
  };
}

/** Apply the two-stage rule described in the module docs. */
export function shouldCollide(a: Filter, b: Filter): boolean {
  if (a.group !== 0 && a.group === b.group) return a.group > 0;
  return (a.mask & b.category) !== 0 && (b.mask & a.category) !== 0;
}

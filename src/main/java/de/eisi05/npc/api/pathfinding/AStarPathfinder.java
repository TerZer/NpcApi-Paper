package de.eisi05.npc.api.pathfinding;

import de.eisi05.npc.api.utils.Var;
import org.bukkit.Location;
import org.bukkit.Material;
import org.bukkit.World;
import org.bukkit.block.Block;
import org.bukkit.block.BlockFace;
import org.bukkit.block.data.Openable;
import org.bukkit.util.BoundingBox;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.*;

public class AStarPathfinder
{
    private final int maxIterations;
    private final boolean allowDiagonal;
    private World world;

    private final PriorityQueue<Node> openSet = new PriorityQueue<>();
    private final Map<Long, Node> allNodes = new HashMap<>();

    public AStarPathfinder(int maxIterations, boolean allowDiagonal)
    {
        this.maxIterations = maxIterations;
        this.allowDiagonal = allowDiagonal;
    }

    public @Nullable List<Location> getPath(@NotNull Location start, @NotNull Location end)
            throws PathfindingUtils.PathfindingException
    {
        if (start.getWorld() == null || end.getWorld() == null)
            return null;

        if (!start.getWorld().equals(end.getWorld()))
            return null;

        openSet.clear();
        allNodes.clear();
        this.world = start.getWorld();

        // Resolve integer FLOOR Y for start/end from feet-based locations (supports stairs/slabs)
        int startFloorY = resolveFloorY(start);
        int endFloorY   = resolveFloorY(end);

        Block startFloor = world.getBlockAt(start.getBlockX(), startFloorY, start.getBlockZ());
        if (!isSafeFloor(startFloor))
            throw new PathfindingUtils.PathfindingException("Start not on a valid floor: " + start);

        Block endFloor = world.getBlockAt(end.getBlockX(), endFloorY, end.getBlockZ());
        if (!isSafeFloor(endFloor))
            throw new PathfindingUtils.PathfindingException("End not on a valid floor: " + end);

        Node startNode = new Node(start.getBlockX(), startFloorY, start.getBlockZ(), null);
        startNode.gCost = 0;
        startNode.calculateH(end);

        openSet.add(startNode);
        allNodes.put(startNode.id, startNode);

        int iterations = 0;

        while (!openSet.isEmpty())
        {
            if (iterations > maxIterations)
                return null;

            iterations++;

            Node current = openSet.poll();

            if (distanceSq(current, end) < 1.0)
                return retracePath(current);

            current.closed = true;

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int z = -1; z <= 1; z++)
                    {
                        if (x == 0 && y == 0 && z == 0)
                            continue;

                        // Only restrict diagonal on X/Z plane (y movement still allowed)
                        if (!allowDiagonal && (Math.abs(x) + Math.abs(z) > 1))
                            continue;

                        int targetX = current.x + x;
                        int targetY = current.y + y; // FLOOR Y changes by y
                        int targetZ = current.z + z;

                        if (!canWalk(current.x, current.y, current.z, targetX, targetY, targetZ))
                            continue;

                        long id = Node.hash(targetX, targetY, targetZ);
                        Node neighbor = allNodes.get(id);
                        if (neighbor == null)
                        {
                            neighbor = new Node(targetX, targetY, targetZ, id);
                            allNodes.put(id, neighbor);
                        }

                        if (neighbor.closed)
                            continue;

                        double moveCost = (Math.abs(x) + Math.abs(y) + Math.abs(z)) > 1 ? 1.41421356237 : 1.0;
                        double newGCost = current.gCost + moveCost;

                        if (newGCost < neighbor.gCost || !openSet.contains(neighbor))
                        {
                            neighbor.gCost = newGCost;
                            neighbor.calculateH(end);
                            neighbor.parent = current;

                            if (!openSet.contains(neighbor))
                                openSet.add(neighbor);
                        }
                    }
                }
            }
        }

        return null;
    }

    /**
     * Advanced physics check.
     * Checks if we can move from (fx, fy, fz) to (tx, ty, tz).
     *
     * Here fy/ty are FLOOR Y (block you stand on),
     * and feet/head occupy ty+1 and ty+2.
     */
    private boolean canWalk(int fx, int fy, int fz, int tx, int ty, int tz)
    {
        Block floor = world.getBlockAt(tx, ty, tz);
        Block spaceFeet = world.getBlockAt(tx, ty + 1, tz);
        Block spaceHead = world.getBlockAt(tx, ty + 2, tz);

        if (!isSafeFloor(floor))
            return false;

        if (isSolid(spaceFeet) || isSolid(spaceHead))
            return false;

        // Corner cutting check for diagonal movement in X/Z
        if (fx != tx && fz != tz)
        {
            Block checkA = world.getBlockAt(fx, ty + 1, tz);
            Block checkB = world.getBlockAt(tx, ty + 1, fz);
            if (isSolid(checkA) || isSolid(checkB))
                return false;
        }

        return true;
    }

    /**
     * Resolve the integer FLOOR Y (block you stand on) from a feet-based Location,
     * using collision boxes to correctly handle stairs/slabs/partial blocks.
     */
    private int resolveFloorY(@NotNull Location loc)
    {
        World w = loc.getWorld();
        if (w == null)
            return loc.getBlockY() - 1;

        int bx = loc.getBlockX();
        int bz = loc.getBlockZ();
        int startY = loc.getBlockY();

        // local coordinates inside block column [0..1)
        double lx = loc.getX() - bx;
        double lz = loc.getZ() - bz;

        // Search downward to find a solid floor candidate
        for (int y = startY + 1; y >= startY - 6; y--)
        {
            Block block = w.getBlockAt(bx, y, bz);

            if (block.getBlockData() instanceof Openable)
                continue;

            if (block.isLiquid())
                continue;

            // must be non-passable solid-ish for floor
            if (!block.getType().isSolid() || block.isPassable())
                continue;

            // If it has collision, accept it as floor
            var boxes = block.getCollisionShape().getBoundingBoxes();
            if (boxes.isEmpty())
                return y;

            // Prefer collision boxes that cover our x/z, otherwise fallback to accepting block
            for (BoundingBox bb : boxes)
            {
                if (lx >= bb.getMinX() && lx <= bb.getMaxX()
                        && lz >= bb.getMinZ() && lz <= bb.getMaxZ())
                {
                    return y;
                }
            }

            // Edge case: standing on boundary -> still accept this block as floor
            return y;
        }

        // Fallback: assume floor is one below current blockY
        return loc.getBlockY() - 1;
    }

    /**
     * Checks if a block is valid to stand ON (floor).
     */
    private boolean isSafeFloor(Block block)
    {
        if (block == null)
            return false;

        Material type = block.getType();
        if (type.isAir() || block.isLiquid())
            return false;

        if (block.isPassable())
            return false;

        return true;
    }


    /**
     * Checks if a block obstructs movement (is a wall).
     */
    private boolean isSolid(Block block)
    {
        if (block == null)
            return false;

        Material type = block.getType();
        if (type.isAir())
            return false;

        // Keep your carpet rules as-is
        if (Var.isCarpet(type) && Var.isCarpet(block.getRelative(BlockFace.UP).getType()))
            return true;

        if (Var.isCarpet(type))
            return false;

        if (block.isPassable())
            return false;

        if (block.getBlockData() instanceof Openable)
            return false;

        return true;
    }

    private @NotNull List<Location> retracePath(@NotNull Node current)
    {
        List<Location> path = new ArrayList<>();
        while (current != null)
        {
            double feetY = feetYAt(current.x, current.y, current.z);
            path.add(new Location(world, current.x + 0.5, feetY, current.z + 0.5));
            current = current.parent;
        }
        Collections.reverse(path);
        return path;
    }

    /**
     * Returns the entity feet Y when standing on top of the floor block at (x,floorY,z),
     * using the collision shape at the block center (0.5, 0.5).
     */
    private double feetYAt(int x, int floorY, int z)
    {
        Block floor = world.getBlockAt(x, floorY, z);
        return floorY + topSurfaceAt(floor, 0.5, 0.5);
    }

    /**
     * Returns the top surface height inside the block [0..1] at local x/z.
     * For a full block -> 1.0
     * For a bottom slab -> 0.5
     * For stairs, etc -> depends on collision shape at that point.
     */
    private double topSurfaceAt(@NotNull Block block, double lx, double lz)
    {
        // If the server returns no boxes, assume full height
        Collection<BoundingBox> boxes = block.getCollisionShape().getBoundingBoxes();
        if (boxes.isEmpty())
            return 1.0;

        double bestTop = -1.0;

        for (BoundingBox bb : boxes)
        {
            // Check if this collision box covers our x/z point
            if (lx >= bb.getMinX() && lx <= bb.getMaxX()
                    && lz >= bb.getMinZ() && lz <= bb.getMaxZ())
            {
                bestTop = Math.max(bestTop, bb.getMaxY());
            }
        }

        // If nothing matched (edge cases), fall back to the maximum box top
        if (bestTop < 0.0)
        {
            for (BoundingBox bb : boxes)
                bestTop = Math.max(bestTop, bb.getMaxY());
        }

        // Safety fallback
        if (bestTop <= 0.0)
            return 1.0;

        return bestTop;
    }

    /**
     * Squared distance from node (center at feet level) to a Location (feet-based).
     */
    private double distanceSq(@NotNull Node n, @NotNull Location l)
    {
        double dx = (n.x + 0.5) - l.getX();
        double dy = feetYAt(n.x, n.y, n.z) - l.getY();
        double dz = (n.z + 0.5) - l.getZ();
        return dx * dx + dy * dy + dz * dz;
    }

    private static class Node implements Comparable<Node>
    {
        final int x, y, z; // y = FLOOR Y
        final long id;

        double gCost = Double.MAX_VALUE;
        double hCost = 0;
        Node parent = null;
        boolean closed = false;

        public Node(int x, int y, int z, Long id)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.id = (id != null) ? id : hash(x, y, z);
        }

        public static long hash(int x, int y, int z)
        {
            return ((long) x & 0x3FFFFFF)
                    | (((long) z & 0x3FFFFFF) << 26)
                    | (((long) y & 0xFFF) << 52);
        }

        public void calculateH(@NotNull Location end)
        {
            double dx = (x + 0.5) - end.getX();
            double dy = (y + 1.0) - end.getY(); // approximate, good enough for heuristic
            double dz = (z + 0.5) - end.getZ();
            this.hCost = Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        public double getFCost()
        {
            return gCost + hCost;
        }

        @Override
        public int compareTo(@NotNull Node other)
        {
            return Double.compare(this.getFCost(), other.getFCost());
        }

        @Override
        public boolean equals(Object o)
        {
            if (this == o) return true;
            if (!(o instanceof Node node)) return false;
            return id == node.id;
        }

        @Override
        public int hashCode()
        {
            return Long.hashCode(id);
        }
    }
}
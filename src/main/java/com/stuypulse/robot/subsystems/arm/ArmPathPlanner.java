package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;

public class ArmPathPlanner {
    private final ArmConfigurationSpace configSpace;
    private final double resolution = 0.1; // Rads

    // A* Node class (inner class)
    private static class Node implements Comparable<Node> {
        final double theta1, theta2;
        double gScore = Double.POSITIVE_INFINITY;
        double fScore = Double.POSITIVE_INFINITY;
        Node parent = null;

        Node(double theta1, double theta2) {
            this.theta1 = theta1;
            this.theta2 = theta2;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fScore, other.fScore);
        }
    }

    public ArmPathPlanner(ArmConfigurationSpace configSpace) {
        this.configSpace = configSpace;
    }

    public List<Translation2d> findPath(double startTheta1, double startTheta2,
                                      double goalTheta1, double goalTheta2) {
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Map<String, Node> allNodes = new HashMap<>();

        // Initialize start node
        Node start = new Node(startTheta1, startTheta2);
        start.gScore = 0;
        start.fScore = heuristic(start, goalTheta1, goalTheta2);
        openSet.add(start);
        allNodes.put(nodeKey(startTheta1, startTheta2), start);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (atGoal(current, goalTheta1, goalTheta2)) {
                return reconstructPath(current);
            }

            // Generate neighbors
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0) continue;

                    double newTheta1 = current.theta1 + i * resolution;
                    double newTheta2 = current.theta2 + j * resolution;
                    String key = nodeKey(newTheta1, newTheta2);

                    if (!configSpace.isValidConfiguration(newTheta1, newTheta2)) continue;

                    Node neighbor = allNodes.getOrDefault(key, new Node(newTheta1, newTheta2));
                    double tentativeGScore = current.gScore + distance(current, neighbor);

                    if (tentativeGScore < neighbor.gScore) {
                        neighbor.parent = current;
                        neighbor.gScore = tentativeGScore;
                        neighbor.fScore = tentativeGScore + heuristic(neighbor, goalTheta1, goalTheta2);
                        
                        if (!openSet.contains(neighbor)) {
                            openSet.add(neighbor);
                        }
                        allNodes.put(key, neighbor);
                    }
                }
            }
        }
        return Collections.emptyList(); // No path
    }

    private List<Translation2d> reconstructPath(Node endNode) {
        LinkedList<Translation2d> path = new LinkedList<>();
        Node current = endNode;
        
        while (current != null) {
            path.addFirst(new Translation2d(current.theta1, current.theta2));
            current = current.parent;
        }
        
        return smoothPath(path);
    }

    private List<Translation2d> smoothPath(List<Translation2d> roughPath) {
        if (roughPath.size() < 3) return roughPath;
        
        List<Translation2d> smoothed = new ArrayList<>();
        smoothed.add(roughPath.get(0));
        
        // Simple averaging filter
        for (int i = 1; i < roughPath.size()-1; i++) {
            double theta1 = (roughPath.get(i-1).getX() + 
                           roughPath.get(i).getX() + 
                           roughPath.get(i+1).getX()) / 3;
            double theta2 = (roughPath.get(i-1).getY() + 
                           roughPath.get(i).getY() + 
                           roughPath.get(i+1).getY()) / 3;
            smoothed.add(new Translation2d(theta1, theta2));
        }
        
        smoothed.add(roughPath.get(roughPath.size()-1));
        return smoothed;
    }

    // Helper methods remain the same as before...
    private boolean atGoal(Node node, double goalTheta1, double goalTheta2) {
        return Math.abs(node.theta1 - goalTheta1) < resolution &&
               Math.abs(node.theta2 - goalTheta2) < resolution;
    }

    private double distance(Node a, Node b) {
        return Math.hypot(b.theta1 - a.theta1, b.theta2 - a.theta2);
    }

    private double heuristic(Node node, double goalTheta1, double goalTheta2) {
        return distance(node, new Node(goalTheta1, goalTheta2));
    }

    private String nodeKey(double theta1, double theta2) {
        return String.format("%.3f,%.3f", theta1, theta2);
    }
}
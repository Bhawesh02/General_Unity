﻿using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PathFinding
{
    public class DStarLite<T>
    {
        private const int MAX_CYCLES = 10000;
        
        private readonly Node<T> m_startNode;
        private readonly Node<T> m_goalNode;
        private readonly List<Node<T>> m_allNodes;
        private readonly SortedSet<(NodeKey, Node<T>)> m_openNodeSet = new(new KeyNodeComparer<T>());
        private readonly Dictionary<Node<T>, NodeKey> m_lockups = new();

        private float m_keyModifier;

        #region NodeSetChanges

        private void InsertNodeSet(NodeKey nodeKey, Node<T> node)
        {
            m_openNodeSet.Add((nodeKey, node));
            m_lockups[node] = nodeKey;
        }
        
        private void RemoveNodeSet(NodeKey nodeKey, Node<T> node)
        {
            m_openNodeSet.Remove((nodeKey, node));
            m_lockups.Remove(node);
        }
         
        private void InsertNodeSet((NodeKey, Node<T>) nodeSet)
        {
            m_openNodeSet.Add(nodeSet);
            m_lockups[nodeSet.Item2] = nodeSet.Item1;
        }
        
        private void RemoveNodeSet((NodeKey, Node<T>) nodeSet)
        {
            m_openNodeSet.Remove(nodeSet);
            m_lockups.Remove(nodeSet.Item2);
        }
        
        private void GetSmallestNode(out Node<T> smallestNode,out NodeKey smallestNodeKey, out NodeKey newKey)
        {
            (NodeKey, Node<T>) smallestNodeSet = m_openNodeSet.Min;
            smallestNode = smallestNodeSet.Item2;
            smallestNodeKey = smallestNodeSet.Item1;
            newKey = CalculateKey(smallestNode);
            RemoveNodeSet(smallestNodeSet);
        }


        #endregion

        
        public DStarLite(Node<T> startNode, Node<T> goalNode, List<Node<T>> allNodes)
        {
            m_startNode = startNode;
            m_goalNode = goalNode;
            m_allNodes = allNodes;
        }
        
        public void Init()
        {
            m_openNodeSet.Clear();
            m_lockups.Clear();
            m_keyModifier = 0f;

            foreach (Node<T> node in m_allNodes)
            {
                node.costToGoalNode = float.MaxValue;
                node.costFromStartNode = float.MaxValue;
            }

            m_goalNode.costFromStartNode = 0f;
            m_goalNode.costToGoalNode = 0f;

            NodeKey key = CalculateKey(m_goalNode);
            InsertNodeSet(key, m_goalNode);
            
        }

        public void ComputeShortestPath()
        {
            int maxSteps = MAX_CYCLES;
            while (m_openNodeSet.Count > 0 && (m_openNodeSet.Min.Item1 < CalculateKey(m_startNode) || m_startNode.costFromStartNode > m_startNode.costToGoalNode))
            {
                if (maxSteps-- <= 0)
                {
                    Debug.LogWarning("Max Steeps Exceeded");
                    break;
                }
                GetSmallestNode(out Node<T> smallestNode, out NodeKey smallestNodeKey, out NodeKey newKey);
                if (smallestNodeKey < newKey)
                {
                    InsertNodeSet(newKey, smallestNode);
                    continue;
                }
                if (smallestNode.costToGoalNode > smallestNode.costFromStartNode)
                {
                    smallestNode.costToGoalNode = smallestNode.costFromStartNode;
                    foreach (Node<T> node in Predecessors(smallestNode))
                    {
                        if (node != m_goalNode)
                        {
                            node.costFromStartNode = Mathf.Min(
                                node.costFromStartNode,
                                node.realCostFromOneNodeToAnother(node, smallestNode) + smallestNode.costToGoalNode
                                );
                        }
                        UpdateVertex(node);
                    }
                    continue;
                }
                float oldCostToGoal = smallestNode.costToGoalNode;
                smallestNode.costToGoalNode = float.MaxValue;
                float costCalculation;
                foreach (Node<T> node in Predecessors(smallestNode).Concat( new[] { smallestNode }))
                {
                    costCalculation = node.realCostFromOneNodeToAnother(node, smallestNode) + oldCostToGoal;
                    if (!Mathf.Approximately(node.costFromStartNode, costCalculation))
                    {
                        continue;
                    }
                    if (node != m_goalNode)
                    {
                        node.costFromStartNode = float.MaxValue;
                    }
                    foreach (Node<T> successor in Successors(node))
                    {
                        node.costFromStartNode = Mathf.Min(
                            node.costFromStartNode,
                            node.realCostFromOneNodeToAnother(node, successor) + successor.costToGoalNode
                        );
                    }
                    UpdateVertex(node);
                }
            }

            m_startNode.costToGoalNode = m_startNode.costFromStartNode;
        }
    
        public void RecalculateNode(Node<T> node)
        {
            m_keyModifier += m_startNode.lowestCostFromOneNodeToAnother(m_startNode, node);

            List<Node<T>> allConnectedNodes = Successors(node).Concat(Predecessors(node)).ToList();

            foreach (Node<T> connectedNode in allConnectedNodes)
            {
                if (connectedNode != m_startNode)
                {
                    connectedNode.costFromStartNode = Mathf.Min(
                        connectedNode.costFromStartNode,
                        connectedNode.realCostFromOneNodeToAnother(connectedNode, node) + node.costToGoalNode
                        );
                }
                UpdateVertex(connectedNode);
                ComputeShortestPath();
            }
            
        }

        public List<Node<T>> GetPath()
        {
            List<Node<T>> newPath = new();
            if (Mathf.Approximately(float.MaxValue, m_startNode.costToGoalNode) ||
                !Mathf.Approximately(m_startNode.costFromStartNode, m_startNode.costToGoalNode))
            {
                Debug.LogWarning("No Path");
                return newPath;
            }
            Node<T> currentNode = m_startNode;
            int maxCycles = MAX_CYCLES;
            float costToGoal;
            while (currentNode != m_goalNode)
            {
                if (--maxCycles == 0)
                {
                    Debug.LogWarning("Max Cycle in Get Path");
                    break;
                }
                newPath.Add(currentNode);
                costToGoal = float.MaxValue;
                Node<T> nextNode = currentNode;
                foreach (Node<T> neighbourNode in Successors(currentNode))
                {
                    if (newPath.Contains(neighbourNode))
                    {
                        continue;
                    }

                    if (neighbourNode == m_goalNode)
                    {
                        nextNode = neighbourNode;
                        break;
                    }
                    if (neighbourNode.costToGoalNode < costToGoal)
                    {
                        costToGoal = neighbourNode.costToGoalNode;
                        nextNode = neighbourNode;
                    }
                }
                currentNode = nextNode;
            }
            newPath.Add(m_goalNode);
            return newPath;
        }
        
        private IEnumerable<Node<T>> Predecessors(Node<T> currentNode)
        {
            return m_allNodes.Where(
                node => node.neighborNodes.Contains(currentNode)
                );
        }
        
        private IEnumerable<Node<T>> Successors(Node<T> currentNode)
        {
            return currentNode.neighborNodes;
        }
        
        private NodeKey CalculateKey(Node<T> node)
        {
            return new NodeKey(
                Mathf.Min(node.costToGoalNode, node.costFromStartNode)
                + node.lowestCostFromOneNodeToAnother(node, m_startNode)
                + m_keyModifier
                ,Mathf.Min(node.costFromStartNode, node.costFromStartNode));
        }
        
        private void UpdateVertex(Node<T> node)
        {
            NodeKey key = CalculateKey(node);
            if (!node.costToGoalEqualFromStart && !m_lockups.ContainsKey(node))
            {
                InsertNodeSet(key, node);
            }
            else if (node.costToGoalEqualFromStart && m_lockups.ContainsKey(node))
            {
                RemoveNodeSet(m_lockups[node], node);
            }
            else if( m_lockups.ContainsKey(node))
            {
                RemoveNodeSet(m_lockups[node], node);
                InsertNodeSet(key, node);
            }
        }
        
    
    }
}
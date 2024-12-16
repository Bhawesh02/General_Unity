using System;
using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
   public class Node<T>
   {
      public T data;
      public readonly Func<Node<T>, Node<T>, float> realCostFromOneNodeToAnother;
      public readonly Func<Node<T>, Node<T>, float> lowestCostFromOneNodeToAnother; // Heuristic
      public float costToGoalNode;
      public float costFromStartNode;
      public bool costToGoalEqualFromStart => Mathf.Approximately(costToGoalNode, costFromStartNode);
      public List<Node<T>> neighborNodes = new();

      public Node(T data, Func<Node<T>, Node<T>, float> realCostFromOneNodeToAnother,
         Func<Node<T>, Node<T>, float> lowestCostFromOneNodeToAnother)
      {
         this.data = data;
         this.realCostFromOneNodeToAnother = realCostFromOneNodeToAnother;
         this.lowestCostFromOneNodeToAnother = lowestCostFromOneNodeToAnother;
         costToGoalNode = float.MaxValue;
         costFromStartNode = float.MaxValue;
      }
   }

}
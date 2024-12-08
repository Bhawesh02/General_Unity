using UnityEngine;

[CreateAssetMenu(fileName = "GridData", menuName = "Grid/GridData", order = 0)]
public class GridData : ScriptableObject
{
    public AxisCombination axisCombination;
    public int row;
    public int column;
    public float slotSpacing;
    public Vector2Int slotSize = Vector2Int.one;
    public Vector2 startPosition;
    public Slot slotPrefab;
}
    
public enum AxisCombination
{
    XY,
    XZ,
    YZ
}
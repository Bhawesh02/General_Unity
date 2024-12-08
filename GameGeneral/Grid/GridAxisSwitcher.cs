using UnityEngine;

public static class GridAxisSwitcher
{
    public static Vector3 GetSwitchedAxisPos(AxisCombination axisCombination, Vector2 coord)
    {

        switch (axisCombination)
        {  
            case AxisCombination.XY:
                return new Vector3(coord.x, coord.y, 0);
            case AxisCombination.XZ:
                return new Vector3(coord.x, 0, coord.y);
            case AxisCombination.YZ:
                return new Vector3(0, coord.x, coord.y);
        }
        return Vector3.zero;
    }

    public static Vector3 GetSwitchedInputPos(AxisCombination axisCombination, Vector3 input)
    {
        switch (axisCombination)
        {
            case AxisCombination.XY:
                return new Vector3(input.x, input.y, 0);
            case AxisCombination.XZ:
                return new Vector3(input.x, input.z, 0);
            case AxisCombination.YZ:
                return new Vector3(0, input.x, input.y);
        }
        return Vector3.zero;
    }
    
}
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class Grid : MonoBehaviour
    {
        [SerializeField] protected GridData m_gridData;

        protected Slot[][] m_slots;
        public Slot[][] Slots
        {
            get { return m_slots; }
        }

        public bool HasSlotItem
        {
            get { return m_gridData.slotPrefab is ItemSlot; }
        }
        
        public int Columns
        {
            get { return m_gridData.column; }
        }
        public int Rows
        {
            get { return m_gridData.row; }
        }

        public float GetSpacedColumns()
        {
            return (m_gridData.slotSpacing + m_gridData.slotSize.x) * m_gridData.column;
        }

        public float GetGetSpacedRows()
        {
            return (m_gridData.slotSpacing + m_gridData.slotSize.y) * m_gridData.row;
        }

        public float GetSpacing()
        {
            return m_gridData.slotSpacing;
        }
        
        public GridData GridData
        {
            get { return m_gridData; }
        }
        
        public Slot this[int x, int y] => GetSlot(x, y);


        #region Private Methods
        
        private void CreateGrid()
        {
            if (m_slots == null)
            {
                m_slots = new Slot[m_gridData.column][];
                for (int i = 0; i < m_gridData.column; i++)
                {
                    m_slots[i] = new Slot[m_gridData.row];
                }
            }

            Vector2 pos = m_gridData.startPosition;
            Slot slot;
            
            for (int i = 0; i < m_gridData.column; ++i)
            {
                pos.x = (i - (m_gridData.column * 0.5f)) * (m_gridData.slotSize.x + m_gridData.slotSpacing);
                pos.x += (m_gridData.slotSize.x * 0.5f);
           
                for (int j = 0; j < m_gridData.row; ++j)
                {
                    pos.y = (j - (m_gridData.row * 0.5f)) * (m_gridData.slotSize.y + m_gridData.slotSpacing);
                    pos.y += (m_gridData.slotSize.y * 0.5f);

                    slot = Instantiate(m_gridData.slotPrefab, transform);
                    slot.name = $"Slot {i},{j}";
                    slot.transform.localPosition = GridAxisSwitcher.GetSwitchedAxisPos(m_gridData.axisCombination,pos);
                    slot.Configure(new Vector2Int(i,j));
                    m_slots[i][j] = slot;
                }
            }
        }
        
        

        
        private void DestroySlot(Slot slot)
        {

    #if UNITY_EDITOR
            if (!EditorApplication.isPlaying && !EditorApplication.isPaused)
            {
                DestroyImmediate(slot.gameObject);
            }
            else
            {
                Destroy(slot.gameObject);
            }
    #else
                Destroy(slot.gameObject);
    #endif

        }
        
        
        private void AddElement(SlotElement element, Vector2Int coord, SlotElementData data)
        {
            if (coord.x >= Columns && coord.y >= Rows)
            {
                return;
            }
            
            element.Configure(m_slots[coord.x][coord.y], data, this);
            ItemSlot slot = (ItemSlot)m_slots[coord.x][coord.y];
            slot.OccupySlot(data,element);
        }
        
        public void Initialize()
        {
            ClearGrid();
            CreateGrid();
        }
        
        public bool IsGridFull()
        {
            int column = m_slots.GetLength(0);
            int row = m_slots.GetLength(1);

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    Slot slot = m_slots[x][y];
                    if (slot.IsEmpty())
                    {
                        return false;
                    }
                }
            }

            return true;
        }
        
        public Slot GetSlot(Vector2Int coord)
        {
            return GetSlot(coord.x, coord.y);
        }
        
        public Slot GetSlot(int x, int y)
        {
            if (x < 0 || y < 0 || x >= m_gridData.column || y >= m_gridData.row)
            {
                return null;
            }
            return m_slots[x][y];
        }
        
        public List<Slot> GetEmptySlots()
        {
            List<Slot> emptySlots = new();
            
            int column = m_gridData.column;
            int row = m_gridData.row;

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    Slot slot = m_slots[x][y];
                    if (slot.IsEmpty())
                    {
                        emptySlots.Add(slot);
                    }
                }
            }

            return emptySlots;
        }
        

        public void OccupySlot(Vector2Int coord, SlotElement element, SlotElementData data)
        {
            AddElement(element,coord,data);
        }
        
        public void EmptySlot(Vector2Int coord)
        {
            if (coord.x >= Columns && coord.y >= Rows)
            {
                return;
            }
            
            Slot slot = m_slots[coord.x][coord.y];
            slot.EmptySlot();
        }
        
        public List<Slot> GetAllOccupiedSlots()
        {
            List<Slot> slots = new List<Slot>(); 
            int column = m_gridData.column;
            int row = m_gridData.row;

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    Slot slot = m_slots[x][y];
                    if (!slot.IsEmpty())
                    {
                        slots.Add(slot);
                    }
                }
            }

            return slots;
        }
        
        public List<Slot> GetAllEmptySlots()
        {
            List<Slot> slots = new List<Slot>(); 
            int column = m_gridData.column;
            int row = m_gridData.row;

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    Slot slot = m_slots[x][y];
                    if (slot.IsEmpty())
                    {
                        slots.Add(slot);
                    }
                }
            }

            return slots;
        }
        
        
        public SlotElement GetElement(Vector3 localPos)
        {
            if (HasSlotItem)
            {
                return null;
            }
            
            float deltaX = (localPos.x / (Columns * m_gridData.slotSpacing)) + 0.5f;
            float deltaY = (localPos.y / (Rows * m_gridData.slotSpacing)) + 0.5f;
        
            int x = Mathf.FloorToInt(deltaX * Columns);
            int y = Mathf.FloorToInt(deltaY * Rows);

            x = Math.Clamp(x, 0, Columns - 1);
            y = Math.Clamp(y, 0, Rows - 1);

            ItemSlot slot = (ItemSlot)m_slots[x][y];
            SlotElement element = slot.SlotElement;
            
            if (element == null)
            {
                return null;
            }

            Vector3 diff = element.transform.localPosition - localPos;
            if (diff.magnitude <= m_gridData.slotSpacing / 2f)
            {
                return element;
            }
            
            return null;
        }
        
        public SlotElement GetElement(int x, int y)
        {
            if (m_slots == null || x < 0 || y < 0 || x >= Columns || y >= Rows)
            {
                return null;
            }
            ItemSlot slot = (ItemSlot)m_slots[x][y];
            SlotElement element = slot.SlotElement;
            return element;
        }

        public List<SlotElement> GetAllElements()
        {
            List<SlotElement> slotsElements = new List<SlotElement>(); 
            int column = m_gridData.column;
            int row = m_gridData.row;

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    Slot slot = m_slots[x][y];
                    if (!slot.IsEmpty())
                    {
                        ItemSlot ItemSlot = (ItemSlot)slot;
                        slotsElements.Add(ItemSlot.SlotElement);
                    }
                }
            }

            return slotsElements;
        }

        public void ClearGrid()
        {
            if (m_slots == null)
            {
                Slot[] slots = GetComponentsInChildren<Slot>();
                if (slots.Length == 0)
                {
                    return;
                }
                
                foreach (Slot slot in slots)
                {
                    DestroyImmediate(slot.gameObject);
                }
                return;
            }
            
            int column = m_gridData.column;
            int row = m_gridData.row;

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    Slot slot = m_slots[x][y];
                    if (slot)
                    {
                        DestroyImmediate(slot.gameObject);
                    }
                    m_slots[x][y] = null;
                }
            }
        }
        
        public void ClearElements()
        {
            if (m_slots == null)
            {
                return;
            }
            
            int column = m_gridData.column;
            int row = m_gridData.row;

            for (int x = 0; x < column; x++)
            {
                for (int y = 0; y < row; y++)
                {
                    m_slots[x][y].EmptySlot();
                }
            }
        }


#if UNITY_EDITOR
        public void InitializeEditor()
        {
            ClearGrid();
            m_slots = null;
            Initialize();
        }

        public void ClearGridEditor()
        {
            ClearGrid();
        }
#endif
        
        #endregion

       
    }
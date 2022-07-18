using UnityEngine;
using System.Collections;
using UnityEngine.UI;


namespace BodyPartSegmentation.UI{
    public class StatusImage : Image
    {
        bool stat = false;
        public bool Stat{get{return stat;}}

        // color when boolean condition is false
        [SerializeField]
        protected Color falseColor = Color.red;
        public Color FalseColor
        {
            get { return falseColor; }
            set { falseColor = value; }
        }

        // color when boolean condition is true
        [SerializeField]
        protected Color trueColor = Color.green;
        public Color TrueColor
        {
            get { return trueColor; }
            set { trueColor = value; }
        }

        public void ToggleImage(bool condition)
        {
            stat = condition;
            if (condition)
            {
                color = TrueColor;
                return;
            }
            color = FalseColor;
        }
    }
}
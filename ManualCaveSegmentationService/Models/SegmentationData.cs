using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;

namespace Service.Models
{
    public class SegmentationData
    {
        public string UploaderName { get; set; }
        public int Expertise { get; set; }
        public int Certainty { get; set; }
        public string Data { get; set; }
        public string UploadDate { get; set; }
        public int Id { get; set; }
    }
}
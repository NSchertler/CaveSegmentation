using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;

namespace Service.Models
{
    /// <summary>
    /// Data for a user's segmentation
    /// </summary>
    public class SegmentationData
    {
        /// <summary>
        /// The author's name
        /// </summary>
        public string UploaderName { get; set; }

        /// <summary>
        /// The author's expertise level
        /// </summary>
        public int Expertise { get; set; }

        /// <summary>
        /// The author's certainty level
        /// </summary>
        public int Certainty { get; set; }

        /// <summary>
        /// Base64-encoded segmentation data
        /// </summary>
        public string Data { get; set; }

        /// <summary>
        /// Date that this segmentation was uploaded
        /// </summary>
        public string UploadDate { get; set; }

        /// <summary>
        /// The id that can be used to query this segmentation.
        /// </summary>
        public int Id { get; set; }
    }
}
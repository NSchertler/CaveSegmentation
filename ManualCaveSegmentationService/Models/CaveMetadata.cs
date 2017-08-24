using System;
using System.Collections.Generic;
using System.Linq;
using System.Web;

namespace Service.Models
{
    /// <summary>
    /// Metadata for a cave
    /// </summary>
    public class CaveMetadata
    {
        /// <summary>
        /// The id that can be used to query this cave.
        /// </summary>
        public int Id { get; set; }

        /// <summary>
        /// The cave's name
        /// </summary>
        public string Name { get; set; }
    }
}
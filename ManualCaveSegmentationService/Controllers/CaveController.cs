using Service.Models;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Threading.Tasks;
using System.Web;
using System.Web.Hosting;
using System.Web.Http;
using System.Xml.Linq;

namespace Service.Controllers
{
    [RoutePrefix("api/Caves")]
    public class CavesController : ApiController
    {
        DirectoryInfo caveDirectory;
        public CavesController()
        {
            caveDirectory = new DirectoryInfo(HostingEnvironment.MapPath("~/App_Data/Caves"));
            if (!caveDirectory.Exists)
                caveDirectory.Create();
        }

        // GET: api/Caves
        public IEnumerable<CaveMetadata> Get()
        {
            var caves = new List<CaveMetadata>();

            string root = HttpContext.Current.Server.MapPath("~/App_Data");
            var cavesDir = new DirectoryInfo(root + "/Caves");

            var subdirs = cavesDir.GetDirectories();
            foreach (var subdir in subdirs)
            {
                try
                {
                    int id = int.Parse(subdir.Name);
                    var metaData = XDocument.Load(subdir.FullName + "/metaData.xml");
                    var rootEl = metaData.Element("Cave");
                    string name = rootEl.Element("Name").Value;

                    caves.Add(new CaveMetadata() { Id = id, Name = name });
                }
                catch (Exception)
                { }
            }

            return caves;            
        }

        // GET: api/Caves/5
        public string Get(int id)
        {
            string root = HttpContext.Current.Server.MapPath("~/App_Data");
            var caveDir = new DirectoryInfo(root + "/Caves/" + id);

            var bytes = File.ReadAllBytes(caveDir.FullName + "/data.bin");

            return Convert.ToBase64String(bytes);
        }


        public async Task<HttpResponseMessage> PostFormData()
        {
            if (!Request.Content.IsMimeMultipartContent())
            {
                throw new HttpResponseException(HttpStatusCode.UnsupportedMediaType);
            }

            string root = HttpContext.Current.Server.MapPath("~/App_Data");
            var provider = new MultipartFormDataStreamProvider(root);

            var cavesDir = new DirectoryInfo(root + "/Caves");
            if (!cavesDir.Exists)
                cavesDir.Create();

            try
            {
                await Request.Content.ReadAsMultipartAsync(provider);

                string name = provider.FormData["name"];
                string pass = provider.FormData["pass"];
                if (pass != name.Substring(name.Length - 3).ToUpper())
                    return Request.CreateErrorResponse(HttpStatusCode.Unauthorized, "The provided password is wrong.");

                var file = provider.FileData[0];

                int id = 1;
                while(new DirectoryInfo(cavesDir.FullName + "/" + id).Exists)
                    ++id;

                var subDir = cavesDir.CreateSubdirectory(id.ToString());
                new FileInfo(file.LocalFileName).MoveTo(subDir.FullName + "/data.bin");

                var metaData = new XDocument();
                var rootEl = new XElement("Cave");
                rootEl.Add(new XElement("Name", name));
                metaData.Add(rootEl);
                metaData.Save(subDir.FullName + "/metaData.xml");

                return Request.CreateResponse(HttpStatusCode.OK);
            }
            catch (System.Exception e)
            {
                return Request.CreateErrorResponse(HttpStatusCode.InternalServerError, e);
            }
        }

        [Route("{caveId}/Segmentations")]
        [HttpPost]
        public HttpResponseMessage PostSegmentation(int caveId, [FromBody] SegmentationData data )
        {
            string root = HttpContext.Current.Server.MapPath("~/App_Data");
            var caveDir = new DirectoryInfo(root + "/Caves/" + caveId);

            var segData = new XDocument();
            var rootEl = new XElement("Segmentation");
            rootEl.Add(new XElement("Uploader", data.UploaderName));
            rootEl.Add(new XElement("Expertise", data.Expertise));
            rootEl.Add(new XElement("Certainty", data.Certainty));
            rootEl.Add(new XElement("SegmentationData", data.Data));
            rootEl.Add(new XElement("UploadDate", DateTime.Now.ToString(CultureInfo.InvariantCulture)));
            segData.Add(rootEl);

            int segId = 1;
            while (new FileInfo(caveDir.FullName + "/seg" + segId + ".xml").Exists)
                segId++;
            segData.Save(caveDir.FullName + "/seg" + segId + ".xml");

            return new HttpResponseMessage(HttpStatusCode.OK);
        }

        [Route("{caveId}/Segmentations")]
        public IEnumerable<SegmentationData> GetSegmentations(int caveId)
        {
            string root = HttpContext.Current.Server.MapPath("~/App_Data");
            var caveDir = new DirectoryInfo(root + "/Caves/" + caveId);

            var segs = new List<SegmentationData>();
            var files = caveDir.GetFiles("seg*.xml");
            foreach (var f in files)
            {
                var xml = XDocument.Load(f.FullName);
                var xmlRoot = xml.Element("Segmentation");
                
                var seg = new SegmentationData();
                seg.Certainty = int.Parse(xmlRoot.Element("Certainty").Value);
                seg.Expertise = int.Parse(xmlRoot.Element("Expertise").Value);
                seg.UploadDate = xmlRoot.Element("UploadDate").Value;
                seg.UploaderName = xmlRoot.Element("Uploader").Value;
                seg.Id = int.Parse(f.Name.Substring(3, f.Name.Length - 7));

                segs.Add(seg);
            }
            return segs;
        }

        [Route("{caveId}/Segmentations/{segId}")]
        public HttpResponseMessage GetSegmentations(int caveId, int segId)
        {
            string root = HttpContext.Current.Server.MapPath("~/App_Data");

            var xml = XDocument.Load(root + "/Caves/" + caveId + "/seg" + segId + ".xml");
            var result = new HttpResponseMessage(HttpStatusCode.OK);
            result.Content = new ByteArrayContent(Convert.FromBase64String(xml.Root.Element("SegmentationData").Value));
            result.Content.Headers.ContentType = new MediaTypeHeaderValue("application/octet-stream");
            result.Content.Headers.ContentDisposition = new ContentDispositionHeaderValue("attachment") { FileName = "segmentation.caveseg" };

            return result;
        }
    }
}

/*
 * Dynamic DMA mapping support.
 *
 * We never have any address translations to worry about, so this
 * is just alloc/free.
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

void *dma_alloc_coherent(struct device *dev, size_t size,
			 dma_addr_t * dma_handle, gfp_t gfp)
{
	void *ret;
	/* ignore region specifiers */
	gfp &= ~(__GFP_DMA | __GFP_HIGHMEM);

	if (dev == NULL || (*dev->dma_mask < 0xffffffff))
		gfp |= GFP_DMA;
	ret = (void *)__get_free_pages(gfp, get_order(size));

	if (ret != NULL) {
		memset(ret, 0, size);
		*dma_handle = (dma_addr_t)ret;
		ret = ioremap((unsigned long)ret, size);
	}
	return ret;
}
EXPORT_SYMBOL(dma_alloc_coherent);

void dma_free_coherent(struct device *dev, size_t size,
		       void *vaddr, dma_addr_t dma_handle)
{
	free_pages((unsigned long)dma_handle, get_order(size));
}
EXPORT_SYMBOL(dma_free_coherent);

/* FIXME: the following dma sync and map need updates */

void dma_sync_single_for_cpu(struct device *dev, dma_addr_t handle,
			     size_t size, enum dma_data_direction dir)
{
}
EXPORT_SYMBOL(dma_sync_single_for_cpu);

void dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg,
			 int nents, enum dma_data_direction dir)
{
}
EXPORT_SYMBOL(dma_sync_sg_for_cpu);

int dma_mapping_error(struct device *dev, dma_addr_t handle)
{
	return 0;
}
EXPORT_SYMBOL(dma_mapping_error);

dma_addr_t dma_map_single(struct device * dev, void *ptr, size_t size,
			  enum dma_data_direction direction)
{
	flush_dcache_range((unsigned long)ptr, (unsigned long)ptr+size);
	return (dma_addr_t)ptr;
}
EXPORT_SYMBOL(dma_map_single);

void dma_unmap_single(struct device *dev, dma_addr_t addr,
		      size_t size, enum dma_data_direction dir)
{
}
EXPORT_SYMBOL(dma_unmap_single);

dma_addr_t dma_map_page(struct device *dev, struct page *page,
			unsigned long offset, size_t size,
			enum dma_data_direction direction)
{
	return page_to_phys(page);
}
EXPORT_SYMBOL(dma_map_page);

void dma_unmap_page(struct device *dev, dma_addr_t address,
		    size_t size, enum dma_data_direction dir)
{
}
EXPORT_SYMBOL(dma_unmap_page);


int
dma_map_sg(struct device *dev, struct scatterlist *sg, int nents,
	   enum dma_data_direction direction)
{
	int i;

	BUG_ON(direction == DMA_NONE);

	for (i = 0; i < nents; i++, sg++) {
		sg->dma_address = (dma_addr_t) sg_virt(sg);

		flush_dcache_range(sg_dma_address(sg),	sg_dma_address(sg) + sg_dma_len(sg));
	}

	return nents;
}
EXPORT_SYMBOL(dma_map_sg);

void dma_unmap_sg(struct device *dev, struct scatterlist *sg,
		int nhwentries, enum dma_data_direction direction)
{
	BUG_ON(direction == DMA_NONE);
}
EXPORT_SYMBOL(dma_unmap_sg);
